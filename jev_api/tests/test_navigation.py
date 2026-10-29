import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choice_control import build_payload, decode_action, ACTION_CHOICES
from geometry.coordinates import xy, EARTH_RADIUS, KNOT_MPS
from geometry.waypoint import WaypointGeometry, circle_interval
from geometry.motion import predict_ground_motion, bounded_turn, braking_facts, preview_duration
from geometry.obstacles import nearby_obstacles, snapshot_clearance
from boat_client import valid_request
from protocol.unity_snapshot import UnitySnapshotAdapter
from protocol.frames import TASK, STOP
from protocol.status import status_content
from protocol.frames import STATUS, json_packets, Reassembler
from geometry.relative_motion import moving_range, relative_facts, line_reference_facts, course_error
COURSES_DEG = tuple(j * 22.5 for j in range(16))

ROOT = Path(__file__).resolve().parents[2]
ORIGIN = [120.5, 30.2]


def geo(x, y):
    return [ORIGIN[0] + math.degrees(x / EARTH_RADIUS / math.cos(math.radians(ORIGIN[1]))),
            ORIGIN[1] + math.degrees(y / EARTH_RADIUS)]


def execution(heading=90, yaw=0):
    return {"motion_model": "bounded_acceleration",
            "max_acceleration_mps2": 1.46, "max_deceleration_mps2": 1.75,
            "max_yaw_rate_deg_s": 12, "max_angular_acceleration_deg_s2": 4,
            "heading_course_deg": heading, "yaw_rate_deg_s": yaw, "fixed_dt_s": 0.02,
            "active_target_course_deg": heading, "active_course_error_deg": 0,
            "hull_length_m": 11.6, "hull_beam_m": 3.4, "obstacle_avoidance_radii_m": {}}


def observation(level=0):
    p = ROOT / "Assets/StreamingAssets" / ("fixed_route.json" if level == 0 else f"Maps/level_{level}.json")
    route = json.loads(p.read_text(encoding="utf-8-sig"))
    return {"model_speed_unit": "mps", "operation": "execute",
            "planning": {"action_duration_s": 1.0, "simulation_paused_while_deciding": True},
            "action_space": {"speeds_knots": [50 * f for f in (0, .05, .1, .2, .35, .5, .75, 1)],
                             "courses_deg": list(COURSES_DEG)},
            "boat": {"longitude": ORIGIN[0], "latitude": ORIGIN[1], "targetSpeed": 0,
                     "targetCourse": None, "collision_radius_m": math.hypot(11.6, 3.4) / 2},
            "execution": execution(),
            "task": {"code": "trackingMode", "planWaypointNum": len(route["waypoints"]) - 1, "next_waypoint_index": 0,
                     "routeSpeed_unit": "m/s",
                     "planWaypointList": [dict(zip(("routeLon", "routeLat"), geo(p["x"], p["y"])),
                                              routeSpeed=p["routeSpeed"]) for p in route["waypoints"][1:]],
                     "arrival_radius_m": route["arrival_radius_m"], "stop_speed_mps": route["stop_speed_mps"]}}


def packet(obs=None):
    return dict(type="decision_request", session_id="s", episode_id="e", run_id="r",
                decision_id="d", tick=0, observation=obs or observation())


def following_facts(payload, name):
    facts = json.loads(payload['questions']['action']['criteria'][name])
    if 'values' in facts:
        values = facts.pop('values')
        facts.update(zip(payload['state']['candidate_value_columns'], values))
    return facts


class NavigationTests(unittest.TestCase):
    def test_geographic_roundtrip(self):
        for actual, expected in zip(xy(geo(1000, 2000), ORIGIN), (1000, 2000)):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_global_cardinal_motion(self):
        for course, expected in [(0, (0, 2)), (90, (2, 0)), (180, (0, -2)), (270, (-2, 0))]:
            m = predict_ground_motion(2, 2, course, execution(course), 1)
            for a, b in zip(m["endpoint"], expected):
                self.assertAlmostEqual(a, b, places=5)

    def test_unknown_course_at_rest_uses_actual_heading(self):
        a = predict_ground_motion(0, 25, 0, execution(90), 1)
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
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 128)

    def test_all_choices_decode_global_physical_commands(self):
        obs = observation()
        for name, (i, j) in ACTION_CHOICES.items():
            result = decode_action({"answers": {"action": {"type": "choice", "choice": name}}}, obs)
            self.assertEqual(result, {"speed": obs["action_space"]["speeds_knots"][i], "course": j * 22.5})
        with self.assertRaises(ValueError):
            decode_action({"answers": {"action": {"type": "choice", "choice": "bad"}}}, obs)

    def test_all_128_choices_and_explicit_risk_descriptions(self):
        obs=observation()
        obs["model_speed_unit"]="knots"
        obs["task"]["next_waypoint_index"]=1
        previous=obs["task"]["planWaypointList"][0]
        obs["boat"].update(longitude=previous["routeLon"],latitude=previous["routeLat"],targetSpeed=10,targetCourse=90)
        obs["obstacles"]=[dict(id="target",longitude=previous["routeLon"],latitude=previous["routeLat"],
                               radius_m=12,avoidance_radius_m=90,targetSpeed=1,targetCourse=0)]
        payload=build_payload(obs)
        self.assertEqual(len(payload["questions"]["action"]["criteria"]),128)
        self.assertNotIn("candidate_legend",payload["state"])
        self.assertIn("speed_1__north_northeast",ACTION_CHOICES)
        self.assertTrue(all("COLLISION RISK"in text and "AVOIDANCE SAFETY CIRCLE WARNING"in text
                            for text in payload["questions"]["action"]["criteria"].values()))

    def test_arrival_multiplier_does_not_enlarge_collision_hull(self):
        obs=observation()
        hull=obs["boat"]["collision_radius_m"]
        self.assertAlmostEqual(obs["task"]["arrival_radius_m"],hull*5)
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
        a = predict_ground_motion(0, 50*KNOT_MPS, 90, execution(), 1)
        self.assertAlmostEqual(a["end_speed"], 1.46)
        self.assertAlmostEqual(a["endpoint"][0], 0.73)
        b = predict_ground_motion(10, 0, 270, execution(), 1)
        self.assertAlmostEqual(b["end_speed"], 8.25)
        self.assertAlmostEqual(b["endpoint"][0], 9.125)

    def test_target_reached_without_speed_overshoot(self):
        m = predict_ground_motion(0, 0.73, 90, execution(), 1)
        self.assertAlmostEqual(m["end_speed"], 0.73)
        self.assertAlmostEqual(m["endpoint"][0], 0.5475, places=4)
        self.assertEqual(len(m["path"]), 51)
        self.assertEqual(len(m["path"]), len(m["path_speeds"]))

    def test_full_speed_stopping_distance_and_braking_advice(self):
        speed = 50*KNOT_MPS
        m = predict_ground_motion(speed, 0, 90, execution(), 16)
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
        full = predict_ground_motion(5, 10, 0, state, 2)
        first = predict_ground_motion(5, 10, 0, state, 1)
        following = dict(state, heading_course_deg=first["end_course_deg"], yaw_rate_deg_s=first["end_yaw_rate_deg_s"],
                         active_target_course_deg=0, active_course_error_deg=-first["end_course_deg"])
        second = predict_ground_motion(first["end_speed"], 10, 0, following, 1)
        for i in range(2):
            self.assertAlmostEqual(first["endpoint"][i] + second["endpoint"][i], full["endpoint"][i], places=6)

    def test_preview_covers_turn_and_braking(self):
        self.assertGreaterEqual(preview_duration(execution(), 1, 50*KNOT_MPS), 21)

    def test_following_omits_only_unused_route_details(self):
        for heading, yaw, target_speed, course in [(180, 0, 3, 0), (359, -4, 0, 90), (0, 8, 12, 315)]:
            state = execution(heading, yaw)
            full = predict_ground_motion(5, target_speed, course, state, 1.03)
            lean = predict_ground_motion(5, target_speed, course, state, 1.03, route_details=False)
            self.assertEqual(lean, {k: v for k, v in full.items() if k not in ('midpoint', 'path_speeds')})
            self.assertIn('midpoint', full)
            self.assertEqual(len(full['path']), len(full['path_speeds']))

    def test_braking_ignores_requested_course(self):
        self.assertEqual(predict_ground_motion(2, 0, 0, execution(90, 4), 1),
                         predict_ground_motion(2, 0, 270, execution(90, 4), 1))

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
        self.assertEqual(len(criteria), 128)

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


class FollowingTests(unittest.TestCase):
    def test_compact_candidate_units_and_warning_schema(self):
        task, records = self.wire()
        self.assertNotIn('arrival_radius_m', task['unity_only'])
        adapter = UnitySnapshotAdapter('mps'); adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        obs = adapter.ready()[0][1]['observation']; before = copy.deepcopy(obs)
        mps = build_payload(obs)
        self.assertEqual(obs, before)
        self.assertNotIn('distance_to_ceiling_m', mps['state']['braking'])
        self.assertNotIn('speed_transition_time_s', mps['state']['execution'])
        obs['model_speed_unit'] = 'knots'
        knots = build_payload(obs)
        self.assertEqual(list(knots['questions']['action']['criteria']), list(ACTION_CHOICES))
        for name in ACTION_CHOICES:
            a = following_facts(mps, name)
            b = following_facts(knots, name)
            self.assertEqual(a['range_m'], b['range_m'])
            self.assertEqual(a['warnings'], b['warnings'])
            self.assertAlmostEqual(a['speed_mps']/KNOT_MPS, b['speed_knots'], delta=.000501)
            self.assertAlmostEqual(a['end_motion']['speed_mps']/KNOT_MPS, b['end_motion']['speed_knots'], delta=.000501)
            for warning in b['warnings']:
                self.assertIn(warning, knots['state']['warning_meanings'])

    def wire(self):
        base = observation()
        task = dict(simId="episode", id="run", code="targetFollowing",
                    params=dict(platCode=1001, targetPlatformId=3001, minFollowingDistanceM=100, maxFollowingDistanceM=200),
                    unity_only=dict(decision_id="decision", tick=0, situation_timestamp=123,
                                    collision_radii_m={"1001": 6.044, "3001": 6, "2001": 3},
                                    max_speed_knots=50, action_duration_s=1,
                                    stop_speed_knots=.1/KNOT_MPS, execution=execution(0)))
        task["unity_only"]["execution"]["obstacle_avoidance_radii_m"] = {"2001": 50}
        records = [dict(base["boat"], platformId=1001, timestamp=123, targetSpeed=10, targetCourse=0)]
        for platform, x, y in [(3001, 0, 150), (2001, 250, 100)]:
            lon, lat = geo(x, y)
            records.append(dict(platformId=platform, timestamp=123, longitude=lon, latitude=lat,
                                targetSpeed=10 if platform == 3001 else 0, targetCourse=0 if platform == 3001 else None))
        return task, records

    def test_target_manifest_role_and_missing_current_target(self):
        task, records = self.wire()
        adapter = UnitySnapshotAdapter("mps")
        adapter.add_task(task, TASK)
        for record in (records[0], records[2]): adapter.add_situation(record)
        self.assertEqual(adapter.ready(), [])
        adapter.add_situation(dict(records[1], timestamp=122))
        self.assertEqual(adapter.ready(), [])
        adapter.add_situation(records[1])
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        self.assertTrue(valid_request(request))
        obs = request["observation"]
        self.assertNotIn("planWaypointList", obs["task"])
        self.assertEqual(obs["target"]["platformId"], 3001)
        self.assertEqual([o["platformId"] for o in obs["obstacles"]], [2001])
        payload = build_payload(obs)
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 128)
        self.assertAlmostEqual(payload["state"]["relative"]["distance_m"], 150)
        self.assertAlmostEqual(payload["state"]["relative"]["range_rate_mps"], 0)
        facts = following_facts(payload, "speed_3__north")
        self.assertEqual(facts["range_m"][0], 150)
        self.assertEqual(facts["reference_error_m"], [0, 0])
        self.assertNotIn("rear_reference", payload["state"])
        self.assertEqual(payload["state"]["task"]["guidance"], "line_of_sight_distance")
        self.assertNotIn("segments", json.dumps(payload))

    def test_motion_sweep_detects_crossing_between_samples(self):
        motion = {"path": [[0, 0], [20, 0]], "sample_dt_s": 1}
        minimum, maximum, end = moving_range(motion, [10, 0], [0, 0], 1)
        self.assertEqual(minimum, 0)
        self.assertEqual(maximum, 10)
        self.assertEqual(end, [-10, 0])
        # Both boats moving equally: range must remain constant, not shrink by own travel.
        self.assertEqual(moving_range(motion, [100, 0], [20, 0], 1)[:2], (100, 100))
        short_final_step = {"path": [[0, 0], [1, 0], [1.5, 0]], "sample_dt_s": 1}
        self.assertEqual(moving_range(short_final_step, [10, 0], [1, 0], 1.5)[:2], (10, 10))
        ahead = relative_facts([-100, 0], [0, 0], [1, 0], [1, 0])
        self.assertEqual(ahead["along_behind_m"], -100)

    def test_following_stop_units_and_stationary_heading(self):
        task, records = self.wire()
        adapter = UnitySnapshotAdapter()
        adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        obs = adapter.ready()[0][1]["observation"]
        obs["obstacles"] = []
        payload = build_payload(obs)
        self.assertEqual(payload["state"]["units"]["speed"], "knots")
        self.assertAlmostEqual(payload["state"]["target"]["actual_speed_knots"], 10)
        # STOP inherits following config but consumes a new complete snapshot.
        stop = copy.deepcopy(task); stop.pop("params")
        stop["unity_only"].update(decision_id="stop", tick=50, situation_timestamp=124)
        adapter.add_task(stop, STOP)
        for record in records:
            changed = dict(record, timestamp=124)
            if record["platformId"] == 3001: changed.update(targetSpeed=0, targetCourse=None)
            adapter.add_situation(changed)
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        self.assertTrue(valid_request(request))
        self.assertEqual(request["observation"]["task"]["reference_course_deg"], 0)
        self.assertEqual(request["observation"]["task"]["code"], "targetFollowing")
        self.assertEqual(request["observation"]["operation"], "stop")
        self.assertNotIn("mode", request["observation"]["task"])
        result = build_payload(request["observation"])
        self.assertIn("IMMEDIATE STOP", result["questions"]["action"]["instructions"])
        self.assertEqual(result["state"]["task"]["mode"], "immediate_stop")
        self.assertNotIn("rear_reference", result["state"])
        self.assertNotIn("line_reference", result["state"])
        self.assertNotIn("recover/maintain", result["questions"]["action"]["instructions"])
        self.assertIn("NONZERO_NOT_STOP", json.loads(result["questions"]["action"]["criteria"]["speed_3__north"])["warnings"])
        self.assertIn("target", result["state"])
        self.assertEqual(len(result["questions"]["action"]["criteria"]), 128)

    def test_status_preserves_wire_identity_and_task_specific_params(self):
        task, _ = self.wire()
        contexts = {(task["simId"], task["id"]): task["params"]}
        stop = copy.deepcopy(task)
        stop.pop("params")
        result = dict(decision_id="stop", tick=50)
        content = status_content(stop, 1, "stopping", contexts, result, 789)
        self.assertEqual(content, dict(simId="episode", id="run", code="targetFollowing",
            status=1, description="stopping", params=dict(targetPlatformId=3001,
                unity_only=dict(decision_id="stop", tick=50, control_timestamp=789))))
        assembler = Reassembler()
        decoded = None
        for data in reversed(json_packets(content, STATUS, stamp=123)):
            decoded = assembler.add(data, ("127.0.0.1", 5005))
        self.assertEqual(decoded, content)
        self.assertNotIn("operation", decoded)
        self.assertNotIn("operation", decoded["params"])

    def test_unimplemented_codes_are_rejected_but_failure_can_be_reported(self):
        for code in ("followMode", "unknown"):
            task, _ = self.wire()
            task["code"] = code
            with self.assertRaises(ValueError):
                UnitySnapshotAdapter().add_task(task, TASK)
            content = status_content(task, 3, "unsupported task", {})
            self.assertEqual(content["code"], code)
            self.assertEqual(content["status"], 3)
            self.assertEqual(content["params"]["currentRoute"], 0)

    def test_invalid_following_constraints_rejected(self):
        for field, value in [("minFollowingDistanceM", float("nan")), ("maxFollowingDistanceM", 50),
                             ("targetPlatformId", 1001), ("targetPlatformId", 9999), ("minFollowingDistanceM", 10)]:
            task, _ = self.wire(); task["params"][field] = value
            with self.assertRaises(ValueError): UnitySnapshotAdapter().add_task(task, TASK)

    def test_initial_minimum_violation_is_recovery_not_new_collision(self):
        task, records = self.wire()
        records[1]["longitude"], records[1]["latitude"] = geo(0, 70)
        adapter = UnitySnapshotAdapter("mps"); adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        obs = adapter.ready()[0][1]["observation"]; obs["obstacles"] = []
        payload = build_payload(obs); question = payload["questions"]["action"]
        self.assertIn("CURRENTLY TOO CLOSE", question["instructions"])
        self.assertIn("SEPARATION_UNCHANGED", json.loads(question["criteria"]["speed_3__north"])["warnings"])
        self.assertIn("SEPARATION_INCREASING", json.loads(question["criteria"]["speed_0__north"])["warnings"])

    def test_collinear_away_recovery_includes_angular_progress(self):
        task, records = self.wire()
        task["unity_only"]["execution"] = execution(180)
        task["unity_only"]["execution"]["obstacle_avoidance_radii_m"] = {"2001": 50}
        records[0].update(targetSpeed=25, targetCourse=180)
        records[1]["longitude"], records[1]["latitude"] = geo(0, 300)
        adapter = UnitySnapshotAdapter("mps"); adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        obs = adapter.ready()[0][1]["observation"]; obs["obstacles"] = []
        payload = build_payload(obs)
        self.assertEqual(payload["state"]["range_recovery"]["phase"], "reorient_before_catchup")
        self.assertEqual(payload["state"]["range_recovery"]["signed_heading_error_deg"], -180)
        self.assertTrue(payload["state"]["range_recovery"]["own_motion_points_away"])
        criteria = payload["questions"]["action"]["criteria"]
        self.assertEqual(len(criteria), 128)
        self.assertIn("COMMAND_POINTS_AWAY", json.loads(criteria["speed_5__south"])["warnings"])
        self.assertNotIn("COMMAND_POINTS_AWAY", json.loads(criteria["speed_5__north"])["warnings"])
        self.assertIn("turn_error_deg", json.loads(criteria["speed_5__north"]))
        self.assertIn("NO_REORIENTATION", json.loads(criteria["speed_0__north"])["warnings"])
        # Short-action range barely distinguishes a useful turn from going away.
        forward = predict_ground_motion(25*KNOT_MPS, 25*KNOT_MPS, 180, execution(180), 1)
        turning = predict_ground_motion(25*KNOT_MPS, 25*KNOT_MPS, 0, execution(180), 1)
        for motion in (forward, turning): motion["sample_dt_s"] = .02
        a = moving_range(forward, (0, 300), (0, 10*KNOT_MPS), 1)[2]
        b = moving_range(turning, (0, 300), (0, 10*KNOT_MPS), 1)[2]
        self.assertGreater(math.hypot(*b), 300)
        self.assertLess(abs(math.hypot(*a)-math.hypot(*b)), .01)
        self.assertLess(abs(course_error(0, turning["end_course_deg"])), abs(course_error(0, forward["end_course_deg"])))
        # Crossing zero degrees must not invent a full-circle correction.
        self.assertEqual(course_error(1, 359), 2)
        self.assertEqual(course_error(359, 1), -2)
        obs["execution"] = execution(70)
        obs["boat"]["targetCourse"] = 70
        self.assertEqual(build_payload(obs)["state"]["range_recovery"]["phase"], "close_range")
        obs["execution"] = execution(180)
        obs["boat"]["targetCourse"] = 180
        # In-band bearing is free; an opposite heading alone must not demand pursuit.
        obs["target"]["longitude"], obs["target"]["latitude"] = geo(0, 150)
        moving_payload = build_payload(obs)
        self.assertEqual(moving_payload["state"]["range_recovery"]["trigger"], "opening_threatens_maximum_before_turn")
        self.assertIn("PREDICTED LOSS OF MAXIMUM RANGE", moving_payload["questions"]["action"]["instructions"])
        obs["boat"].update(targetSpeed=0, targetCourse=None)
        obs["target"].update(targetSpeed=0, targetCourse=None)
        self.assertNotIn("range_recovery", build_payload(obs)["state"])
        obs["target"]["longitude"], obs["target"]["latitude"] = geo(0, 300)
        obs["operation"] = "stop"
        obs["task"]["stop_speed_mps"] = .1
        self.assertNotIn("range_recovery", build_payload(obs)["state"])

    def test_reference_on_current_line_not_target_heading(self):
        point = line_reference_facts([180, 240], 150)
        self.assertEqual((point["east_m"], point["north_m"]), (90, 120))
        self.assertEqual(point["range_error_m"], 150)
        self.assertAlmostEqual(point["away_course_deg"], 216.86989764584402)
        at_range = line_reference_facts([150, 0], 150)
        self.assertEqual((at_range["east_m"], at_range["north_m"]), (0, 0))
        self.assertFalse(line_reference_facts([0, 0], 150)["defined"])

    def test_incoming_target_needs_active_escape_not_parking(self):
        task, records = self.wire()
        records[0].update(targetSpeed=0, targetCourse=None)
        records[1].update(longitude=geo(0, 70)[0], latitude=geo(0, 70)[1], targetCourse=180)
        task["unity_only"]["execution"] = execution(180)
        task["unity_only"]["execution"]["obstacle_avoidance_radii_m"] = {"2001": 50}
        adapter = UnitySnapshotAdapter("mps"); adapter.add_task(task, TASK)
        for record in records: adapter.add_situation(record)
        obs = adapter.ready()[0][1]["observation"]; obs["obstacles"] = []
        payload = build_payload(obs); question = payload["questions"]["action"]
        self.assertTrue(payload["state"]["relative"]["target_approaching_own_position"])
        self.assertAlmostEqual(payload["state"]["relative"]["range_rate_mps"], -10*KNOT_MPS)
        self.assertEqual(payload["state"]["line_reference"]["away_course_deg"], 180)
        self.assertIn("actively MOVE AWAY", question["instructions"])
        self.assertIn("TARGET_COLLISION", json.loads(question["criteria"]["speed_0__south"])["warnings"])
        self.assertNotIn("TARGET_COLLISION", json.loads(question["criteria"]["speed_4__south"])["warnings"])
        self.assertEqual(len(question["criteria"]), 128)


if __name__ == "__main__":
    unittest.main()
