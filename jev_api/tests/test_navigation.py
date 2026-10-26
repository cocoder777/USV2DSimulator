import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choice_control import build_payload, decode_action, ACTION_CHOICES
from tools.waypoint_geometry import xy, EARTH_RADIUS, KNOT_MPS, WaypointGeometry, circle_interval
from tools.navigation_motion import predict_ground_motion, bounded_turn, braking_facts, preview_duration
from tools.obstacle_geometry import nearby_obstacles, snapshot_clearance
from boat_client import valid_request
from tools.client_protocol import UnitySnapshotAdapter, TASK
COURSES_DEG = tuple(range(0,360,45))

ROOT = Path(__file__).resolve().parents[2]
ORIGIN = [120.5, 30.2]


def geo(x, y):
    return [ORIGIN[0] + math.degrees(x / EARTH_RADIUS / math.cos(math.radians(ORIGIN[1]))),
            ORIGIN[1] + math.degrees(y / EARTH_RADIUS)]


def execution(heading=90, yaw=0):
    return {"motion_model": "bounded_acceleration",
            "speed_transition_time_s": 0, "course_transition_time_s": 0,
            "max_acceleration_mps2": 1.46, "max_deceleration_mps2": 1.75,
            "max_yaw_rate_deg_s": 12, "max_angular_acceleration_deg_s2": 4,
            "heading_course_deg": heading, "yaw_rate_deg_s": yaw, "fixed_dt_s": 0.02,
            "active_target_course_deg": heading, "active_course_error_deg": 0,
            "hull_length_m": 11.6, "hull_beam_m": 3.4, "obstacle_avoidance_radii_m": {}}


def observation(level=0):
    p = ROOT / "Assets/StreamingAssets" / ("fixed_route.json" if level == 0 else f"Maps/level_{level}.json")
    route = json.loads(p.read_text(encoding="utf-8-sig"))
    return {"model_speed_unit": "mps",
            "planning": {"action_duration_s": 1.0, "simulation_paused_while_deciding": True},
            "action_space": {"speeds_knots": [50 * f for f in (0, .05, .1, .2, .35, .5, .75, 1)],
                             "courses_deg": list(COURSES_DEG)},
            "boat": {"longitude": ORIGIN[0], "latitude": ORIGIN[1], "targetSpeed": 0,
                     "targetCourse": None, "collision_radius_m": math.hypot(11.6, 3.4) / 2},
            "execution": execution(),
            "task": {"planWaypointNum": len(route["waypoints"]) - 1, "next_waypoint_index": 0,
                     "routeSpeed_unit": "m/s",
                     "planWaypointList": [dict(zip(("routeLon", "routeLat"), geo(p["x"], p["y"])),
                                              routeSpeed=p["routeSpeed"]) for p in route["waypoints"][1:]],
                     "arrival_radius_m": route["arrival_radius_m"], "stop_speed_mps": route["stop_speed_mps"]}}


def packet(obs=None):
    return dict(type="decision_request", session_id="s", episode_id="e", run_id="r",
                decision_id="d", tick=0, observation=obs or observation())


class NavigationTests(unittest.TestCase):
    def test_geographic_roundtrip(self):
        for actual, expected in zip(xy(geo(1000, 2000), ORIGIN), (1000, 2000)):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_global_cardinal_motion(self):
        for course, expected in [(0, (0, 2)), (90, (2, 0)), (180, (0, -2)), (270, (-2, 0))]:
            m = predict_ground_motion(2, course, 2, course, execution(course), 1)
            for a, b in zip(m["endpoint"], expected):
                self.assertAlmostEqual(a, b, places=5)

    def test_unknown_course_at_rest_uses_actual_heading(self):
        a = predict_ground_motion(0, None, 25, 0, execution(90), 1)
        self.assertGreater(a["endpoint"][0], 0.72)
        self.assertLess(abs(a["endpoint"][1]), 0.03)
        p = build_payload(observation())
        self.assertIn("measured heading/yaw", p["state"]["execution"]["forecast"])
        self.assertTrue(valid_request(packet()))

    def test_no_reference_route_or_local_action_selection(self):
        obs = observation()
        payload = build_payload(obs)
        self.assertNotIn("route", payload["state"])
        self.assertNotIn("reference_lon_lat", json.dumps(payload))
        self.assertAlmostEqual(payload["state"]["target"]["distance_m"], 400, places=3)
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 64)

    def test_all_choices_decode_global_physical_commands(self):
        obs = observation()
        for name, (i, j) in ACTION_CHOICES.items():
            result = decode_action({"answers": {"action": {"type": "choice", "choice": name}}}, obs)
            self.assertEqual(result, {"speed": obs["action_space"]["speeds_knots"][i], "course": j * 45})
        with self.assertRaises(ValueError):
            decode_action({"answers": {"action": {"type": "choice", "choice": "bad"}}}, obs)

    def test_original_64_choices_and_explicit_risk_descriptions(self):
        obs=observation()
        obs["model_speed_unit"]="knots"
        obs["task"]["next_waypoint_index"]=1
        previous=obs["task"]["planWaypointList"][0]
        obs["boat"].update(longitude=previous["routeLon"],latitude=previous["routeLat"],targetSpeed=10,targetCourse=90)
        obs["obstacles"]=[dict(id="target",longitude=previous["routeLon"],latitude=previous["routeLat"],
                               radius_m=12,avoidance_radius_m=90,targetSpeed=1,targetCourse=0)]
        payload=build_payload(obs)
        self.assertEqual(len(payload["questions"]["action"]["criteria"]),64)
        self.assertNotIn("candidate_legend",payload["state"])
        self.assertNotIn("speed_1__north_northeast",ACTION_CHOICES)
        self.assertTrue(all("COLLISION RISK"in text and "AVOIDANCE SAFETY CIRCLE WARNING"in text
                            for text in payload["questions"]["action"]["criteria"].values()))

    def test_arrival_multiplier_does_not_enlarge_collision_hull(self):
        obs=observation()
        hull=obs["boat"]["collision_radius_m"]
        self.assertAlmostEqual(obs["task"]["arrival_radius_m"],hull*10)
        self.assertAlmostEqual(hull,6.0440052945)

    def test_protocol_rejects_invalid_limits_state_and_timing(self):
        for key, value in [("max_deceleration_mps2", 0), ("max_acceleration_mps2", -1),
                           ("heading_course_deg", 360), ("yaw_rate_deg_s", 13), ("fixed_dt_s", float("nan"))]:
            request = packet()
            request["observation"]["execution"][key] = value
            self.assertFalse(valid_request(request))
        request = packet()
        del request["observation"]["boat"]["collision_radius_m"]
        self.assertFalse(valid_request(request))
        request = packet()
        request["observation"]["planning"]["action_duration_s"] = 0.031
        self.assertFalse(valid_request(request))
        request = packet()
        request["observation"]["boat"].update(targetSpeed=1, targetCourse=None)
        self.assertFalse(valid_request(request))

    def test_waypoint_speed_is_ceiling(self):
        obs = observation()
        payload = build_payload(obs)
        self.assertEqual(payload["state"]["target"]["passing_speed_ceiling_mps"],
                         obs["task"]["planWaypointList"][0]["routeSpeed"])
        self.assertIn("only a ceiling", payload["state"]["task"]["background"])

    def test_circular_hull_swept_contact_and_speed(self):
        geometry = WaypointGeometry(observation())
        geometry.radius = math.hypot(11.6, 3.4) / 2
        radius = geometry.radius
        facts = geometry.motion_facts({"path": [[-2*radius, 0], [2*radius, 0]],
            "path_speeds": [4, 0], "endpoint": [2*radius, 0]}, waypoint_index=0)
        self.assertFalse(facts["enters_checkpoint"])
        geometry.waypoints = [(0, 0)]
        facts = geometry.motion_facts({"path": [[-2*radius, 0], [2*radius, 0]],
            "path_speeds": [4, 0], "endpoint": [2*radius, 0]})
        self.assertTrue(facts["enters_checkpoint"])
        self.assertAlmostEqual(facts["max_speed_in_circle_mps"], 3)
        self.assertIsNone(circle_interval((-20, radius + 1), (20, radius + 1), (0, 0), radius))

    def test_passed_circle_ceiling_stays_visible(self):
        obs = observation(4)
        obs["task"]["next_waypoint_index"] = 1
        previous = obs["task"]["planWaypointList"][0]
        obs["boat"].update(longitude=previous["routeLon"], latitude=previous["routeLat"], targetSpeed=1/KNOT_MPS, targetCourse=90)
        p = build_payload(obs)
        self.assertEqual(p["state"]["recently_passed_circle"]["speed_ceiling_mps"], previous["routeSpeed"])

    def test_final_parking_stage(self):
        obs = observation()
        obs["task"]["next_waypoint_index"] = obs["task"]["planWaypointNum"] - 1
        end = obs["task"]["planWaypointList"][-1]
        obs["boat"].update(longitude=end["routeLon"], latitude=end["routeLat"])
        self.assertEqual(build_payload(obs)["state"]["task"]["stage"], "final_parking")

    def test_acceleration_and_asymmetric_braking(self):
        a = predict_ground_motion(0, None, 50*KNOT_MPS, 90, execution(), 1)
        self.assertAlmostEqual(a["end_speed"], 1.46)
        self.assertAlmostEqual(a["endpoint"][0], 0.73)
        b = predict_ground_motion(10, 90, 0, 270, execution(), 1)
        self.assertAlmostEqual(b["end_speed"], 8.25)
        self.assertAlmostEqual(b["endpoint"][0], 9.125)

    def test_target_reached_without_speed_overshoot(self):
        m = predict_ground_motion(0, None, 0.73, 90, execution(), 1)
        self.assertAlmostEqual(m["end_speed"], 0.73)
        self.assertAlmostEqual(m["endpoint"][0], 0.5475, places=4)
        self.assertEqual(len(m["path"]), 51)
        self.assertEqual(len(m["path"]), len(m["path_speeds"]))

    def test_full_speed_stopping_distance_and_braking_advice(self):
        speed = 50*KNOT_MPS
        m = predict_ground_motion(speed, 90, 0, 90, execution(), 16)
        self.assertEqual(m["end_speed"], 0)
        self.assertAlmostEqual(m["endpoint"][0], speed**2/(2*1.75), places=4)
        facts = braking_facts(speed, execution(), 0)
        self.assertAlmostEqual(facts["time_to_stop_s"], speed/1.75)

    def test_turn_limits_and_rest_to_rest_time(self):
        self.assertEqual(bounded_turn(90, 0, 12, 4, 1), (2, 4))
        self.assertEqual(bounded_turn(90, 0, 12, 4, 3), (18, 12))
        self.assertEqual(bounded_turn(90, 0, 12, 4, 10.5), (90, 0))
        previous = 0
        for tick in range(1, 526):
            _, yaw = bounded_turn(90, 0, 12, 4, tick * .02)
            self.assertLessEqual(abs(yaw), 12 + 1e-7)
            self.assertLessEqual(abs(yaw - previous), .08 + 1e-7)
            previous = yaw

    def test_reversing_turn_preserves_inertia(self):
        angle, yaw = bounded_turn(-90, 12, 12, 4, 1)
        self.assertAlmostEqual(yaw, 8)
        self.assertAlmostEqual(angle, 10)

    def test_repeated_command_continues_angular_state(self):
        state = execution(90, -5)
        full = predict_ground_motion(5, 90, 10, 0, state, 2)
        first = predict_ground_motion(5, 90, 10, 0, state, 1)
        following = dict(state, heading_course_deg=first["end_course_deg"], yaw_rate_deg_s=first["end_yaw_rate_deg_s"],
                         active_target_course_deg=0, active_course_error_deg=-first["end_course_deg"])
        second = predict_ground_motion(first["end_speed"], first["end_course_deg"], 10, 0, following, 1)
        for i in range(2):
            self.assertAlmostEqual(first["endpoint"][i] + second["endpoint"][i], full["endpoint"][i], places=6)

    def test_preview_covers_turn_and_braking(self):
        self.assertGreaterEqual(preview_duration(execution(), 1, 50*KNOT_MPS), 21)

    def test_braking_ignores_requested_course(self):
        self.assertEqual(predict_ground_motion(2, 90, 0, 0, execution(90, 4), 1),
                         predict_ground_motion(2, 90, 0, 270, execution(90, 4), 1))

    def test_snapshot_adapter_preserves_units_and_complete_manifest(self):
        obs = observation()
        params = dict(platCode=1001, planRouteNum=0, currentRoute=0,
                      planWaypointNum=obs["task"]["planWaypointNum"],
                      planWaypointList=[dict(p, routeSpeed=p["routeSpeed"]/KNOT_MPS)
                                        for p in obs["task"]["planWaypointList"]])
        task = dict(simId="episode", id="run", code="trackingMode", params=params,
                    unity_only=dict(decision_id="decision", tick=0, situation_timestamp=123,
                                    collision_radii_m={"1001":obs["boat"]["collision_radius_m"], "2001":3},
                                    max_speed_knots=50, action_duration_s=1,
                                    arrival_radius_m=obs["task"]["arrival_radius_m"],
                                    stop_speed_knots=.1/KNOT_MPS, execution=execution()))
        task["unity_only"]["execution"]["obstacle_avoidance_radii_m"]={"2001":50}
        own=dict(obs["boat"], platformId=1001, timestamp=123)
        lon,lat=geo(200,0)
        obstacle=dict(platformId=2001,timestamp=123,longitude=lon,latitude=lat,targetSpeed=0,targetCourse=None)
        adapter=UnitySnapshotAdapter("mps")
        self.assertTrue(adapter.add_task(task,TASK))
        adapter.add_situation(own)
        self.assertEqual(adapter.ready(),[])
        adapter.add_situation(obstacle)
        ready=adapter.ready()
        self.assertEqual(len(ready),1)
        adapted=ready[0][1]
        self.assertIsNone(ready[0][2])
        self.assertTrue(valid_request(adapted))
        self.assertEqual(adapted["observation"]["action_space"]["speeds_knots"][-1],50)
        self.assertEqual(adapted["observation"]["execution"]["heading_course_deg"],90)
        self.assertEqual(adapted["observation"]["obstacles"][0]["avoidance_radius_m"],50)
        self.assertEqual(adapted["observation"]["task"]["planWaypointList"][0]["routeSpeed"],
                         obs["task"]["planWaypointList"][0]["routeSpeed"])
        self.assertEqual(adapter.situations,{})
        self.assertFalse(adapter.add_task(task,TASK))

    def test_safety_and_collision_circles_are_distinct(self):
        obs = observation()
        lon, lat = geo(100, 0)
        obs["obstacles"] = [{"id": "large", "longitude": lon, "latitude": lat, "radius_m": 12,
                             "avoidance_radius_m": 90, "targetSpeed": 0, "targetCourse": None}]
        obstacles = nearby_obstacles(obs)
        self.assertEqual(len(obstacles), 1)
        path = [[0, 0], [20, 0]]
        hull = obs["boat"]["collision_radius_m"]
        self.assertGreater(snapshot_clearance(path, obstacles, hull), 0)
        self.assertLess(snapshot_clearance(path, obstacles, hull, "avoidance_radius_m"), 0)

    def test_stopped_boat_obstacle_prompt_does_not_describe_braking_as_bypass(self):
        # Regression: stopped ~64 m before a small obstacle, with a safe low-speed turn.
        obs = observation()
        obs["execution"] = execution(106)
        lon, lat = geo(64, 8)
        obs["obstacles"] = [{"id": "small", "longitude": lon, "latitude": lat,
                             "radius_m": 3, "avoidance_radius_m": 50,
                             "targetSpeed": 0, "targetCourse": None}]
        payload = build_payload(obs)
        criteria = payload["questions"]["action"]["criteria"]
        for name, (speed, _) in ACTION_CHOICES.items():
            if speed == 0:
                self.assertIn("Braking only", criteria[name])
                self.assertNotIn("Choose this movement to start", criteria[name])
        self.assertIn("nonzero safe bypass", payload["questions"]["action"]["instructions"])
        self.assertNotIn("SAFETY CIRCLE WARNING", criteria["speed_1__south"])
        self.assertEqual(len(criteria), 64)

    def test_all_maps_request_validation_and_units(self):
        for level in range(5):
            obs = observation(level)
            self.assertTrue(valid_request(packet(obs)))
            original = copy.deepcopy(obs)
            mps = build_payload(obs)
            self.assertEqual(obs, original)
            obs["model_speed_unit"] = "knots"
            knots = build_payload(obs)
            self.assertEqual(knots["state"]["units"]["speed"], "knots")
            self.assertEqual(list(mps["questions"]["action"]["criteria"]), list(knots["questions"]["action"]["criteria"]))


if __name__ == "__main__":
    unittest.main()
