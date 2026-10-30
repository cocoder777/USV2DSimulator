import copy
import json
import math
import re
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
COURSES_DEG = tuple(range(0, 360, 45))

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
    if isinstance(facts, list):
        return dict(zip(payload['state']['candidate_value_columns'], facts))
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
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 64)

    def test_all_choices_decode_global_physical_commands(self):
        obs = observation()
        for name, (i, j) in ACTION_CHOICES.items():
            result = decode_action({"answers": {"action": {"type": "choice", "choice": name}}}, obs)
            self.assertEqual(result, {"speed": obs["action_space"]["speeds_knots"][i], "course": j * 45})
        with self.assertRaises(ValueError):
            decode_action({"answers": {"action": {"type": "choice", "choice": "bad"}}}, obs)

    def test_all_64_choices_and_explicit_risk_descriptions(self):
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


class FollowingTests(unittest.TestCase):
    def wire(self):
        base = observation()
        task = dict(simId='episode', id='run', code='followMode',
                    params=dict(platCode=1001, targetPlatCode=3001,
                                followDistanceMinM=100, followDistanceMaxM=200, maxSpeed=0),
                    unity_only=dict(decision_id='decision', tick=0, situation_timestamp=123,
                                    collision_radii_m={'1001': 6.044, '3001': 6, '2001': 3},
                                    max_speed_knots=50, action_duration_s=1, target_reference_course_deg=0,
                                    stop_speed_knots=.1/KNOT_MPS, execution=execution(0)))
        task['unity_only']['execution']['obstacle_avoidance_radii_m'] = {'2001': 50}
        records = [dict(base['boat'], platformId=1001, timestamp=123, targetSpeed=10, targetCourse=0)]
        for platform, x, y in [(3001, 0, 150), (2001, 250, 100)]:
            lon, lat = geo(x, y)
            records.append(dict(platformId=platform, timestamp=123, longitude=lon, latitude=lat,
                                targetSpeed=10 if platform == 3001 else 0,
                                targetCourse=0 if platform == 3001 else None))
        return task, records

    def adapted(self, task=None, records=None, unit='mps'):
        if task is None:
            task, records = self.wire()
        adapter = UnitySnapshotAdapter(unit)
        adapter.add_task(task, TASK)
        for record in records:
            adapter.add_situation(record)
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        self.assertTrue(valid_request(request))
        return adapter, request['observation']

    def test_rear_sector_is_a_closed_half_annulus_not_a_lateral_strip(self):
        from geometry.following import sector_facts
        for q, expected in [((0, -150), True), ((140, -30), True), ((150, 0), True),
                            ((0, 150), False), ((0, -99), False), ((0, -201), False)]:
            facts = sector_facts(q, 0, 100, 200)
            self.assertEqual(facts['in_sector'], expected)
            self.assertEqual(facts['sector_error_m'] < 1e-8, expected)
        self.assertEqual(sector_facts((-150, 0), 90, 100, 200)['relative_bearing_deg'], 180)
        self.assertAlmostEqual(sector_facts((0, 150), 0, 100, 200)['sector_error_m'], math.hypot(150, 100))
        self.assertEqual(sector_facts((0, -50), 0, 100, 200)['distance_out_m'], -50)
        core = sector_facts((140, -30), 0, 125, 175, 45)
        self.assertFalse(core['in_sector'])
        self.assertGreater(core['sector_error_m'], 0)
        self.assertEqual(sector_facts((0, -150), 0, 125, 175, 45)['sector_error_m'], 0)

    def test_target_manifest_needs_the_current_complete_snapshot(self):
        task, records = self.wire()
        adapter = UnitySnapshotAdapter('mps')
        adapter.add_task(task, TASK)
        for record in (records[0], records[2]): adapter.add_situation(record)
        self.assertEqual(adapter.ready(), [])
        adapter.add_situation(dict(records[1], timestamp=122))
        self.assertEqual(adapter.ready(), [])
        adapter.add_situation(records[1])
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        obs = request['observation']
        self.assertEqual([o['id'] for o in obs['obstacles']], ['2001'])
        payload = build_payload(obs)
        self.assertEqual(len(payload['questions']['action']['criteria']), 128)
        self.assertEqual(payload['state']['task']['mode'], 'maintain')
        self.assertAlmostEqual(payload['state']['tracking_error_m'], 0)
        self.assertNotIn('segments', json.dumps(payload))

    def test_short_candidate_text_preserves_units_geometry_and_obstacle_clearance(self):
        _, obs = self.adapted()
        # Put an existing obstacle close enough to include its swept facts.
        obs['obstacles'][0]['longitude'], obs['obstacles'][0]['latitude'] = geo(60, 50)
        before = copy.deepcopy(obs)
        mps = build_payload(obs)
        self.assertEqual(obs, before)
        obs['model_speed_unit'] = 'knots'
        knots = build_payload(obs)
        self.assertEqual(len(knots['questions']['action']['criteria']), 128)
        for name in mps['questions']['action']['criteria']:
            a, b = (p['questions']['action']['criteria'][name] for p in (mps,knots))
            speed_pattern = r'(-?\d+\.\d+) (?:knots|m/s)'
            speeds_a, speeds_b = (list(map(float,re.findall(speed_pattern,text))) for text in (a,b))
            self.assertEqual(len(speeds_a), 1)
            for va,vb in zip(speeds_a,speeds_b):
                self.assertAlmostEqual(va/KNOT_MPS,vb,delta=.0011)
            self.assertEqual(re.sub(speed_pattern,'<speed>',a), re.sub(speed_pattern,'<speed>',b))
            self.assertIn('Hull clearance',b)
            self.assertIn('Safety clearance',b)
        self.assertLess(len(json.dumps(knots).encode('utf-8')), 65000)

    def test_region_requests_distinguish_maintenance_and_loss(self):
        from geometry.following import FollowingGeometry
        _, obs = self.adapted()
        texts = []
        for distance, course, speed, phase in [(150, 0, 10, 'maintain'), (70, 180, 10, 'recover'),
                                               (300, 0, 10, 'recover'), (150, 0, 0, 'maintain')]:
            obs['target'].update(longitude=geo(0,distance)[0], latitude=geo(0,distance)[1], targetCourse=course, targetSpeed=speed)
            obs['task']['reference_course_deg'] = course
            obs['task'].pop('following_request',None)
            payload = build_payload(obs)
            texts.append(payload['questions']['action']['instructions'])
            self.assertEqual(payload['state']['task']['mode'], phase)
            self.assertEqual('bypass_remaining_m' in payload['state'], phase == 'recover')
            self.assertNotIn('request_variant',obs['task'])
            self.assertNotIn('interior_control_region',payload['state'])
        self.assertEqual(len(set(texts)),2)
        self.assertTrue(all(len(t)<1100 for t in texts))
        # Same centre distance can mean zero error astern or large error ahead.
        obs['target'].update(longitude=geo(0,150)[0],latitude=geo(0,150)[1])
        obs['task']['reference_course_deg']=0
        self.assertAlmostEqual(FollowingGeometry(obs).current['sector_error_m'],0)
        obs['task']['reference_course_deg']=180
        self.assertGreater(FollowingGeometry(obs).current['sector_error_m'],150)

    def test_tracking_forecast_separates_progress_from_target_contact(self):
        from tasks.task2_following import build_payload as raw_payload
        _,obs = self.adapted()
        # Own boat faces a stationary target, whose rear point is beyond its hull.
        # Pursuing that point directly must still expose swept contact risk.
        obs['target'].update(targetSpeed=0)
        obs['task']['reference_course_deg']=180
        obs['task'].pop('following_request',None)
        payload=raw_payload(obs)
        self.assertEqual(len(payload['questions']['action']['criteria']),128)
        choices=list(payload['questions']['action']['criteria'].values())
        self.assertTrue(any(c['hull_clearance_m'][2]<0 for c in choices))
        self.assertTrue(all('region_error_m' in c and 'safety_clearance_m' in c for c in choices))
        self.assertTrue(all(len(c['safety_clearance_m'])==4 for c in choices))

    def test_recovery_geometry_handles_both_sides_inside_protection_and_degenerate_band(self):
        from geometry.following import FollowingGeometry, request_phase
        _, obs = self.adapted()
        geometry = FollowingGeometry(obs)
        ahead = geometry.bypass_distances((0,150))
        self.assertAlmostEqual(ahead[0],ahead[1])
        # Bypassing the disk is longer than the straight distance to the set.
        self.assertGreater(ahead[0],geometry.region((0,150))['sector_error_m'])
        self.assertEqual(geometry.bypass_distances((0,-150)),[0,0])
        right, left = (geometry.bypass_distances(q) for q in ((140,50),(-140,50)))
        self.assertAlmostEqual(right[0],left[1])
        self.assertAlmostEqual(right[1],left[0])
        # A stopped target ahead requires a tangent detour, never a reference
        # through its protection disk. Mirror symmetry must survive rotation.
        geometry.speed=0
        lengths,courses=geometry.bypass((0,150))
        self.assertTrue(90<courses[0]<180)
        self.assertTrue(180<courses[1]<270)
        self.assertAlmostEqual(courses[0]+courses[1],360)
        geometry.course=90
        rotated_lengths,rotated_courses=geometry.bypass((150,0))
        self.assertEqual(lengths,rotated_lengths)
        for a,b in zip(courses,rotated_courses): self.assertAlmostEqual((b-a)%360,90)
        geometry.course=0
        from geometry.relative_motion import direction
        _,boundary_courses=geometry.bypass(tuple(100*x for x in direction(270)))
        self.assertTrue(all(c is None or 0<=c<360 for c in boundary_courses))
        for lower,upper in ((0,200),(100,200),(150,150)):
            obs['task'].update(followDistanceMinM=lower,followDistanceMaxM=upper)
            g=FollowingGeometry(obs)
            for q in ((0,0),(0,5),(0,100),(1000,0),(0,-300)):
                self.assertTrue(all(math.isfinite(v) and v>=0 for v in g.bypass_distances(q)))
        obs['task'].update(followDistanceMinM=100,followDistanceMaxM=200)
        geometry.current.update(in_sector=False,sector_error_m=15,distance_m=215,along_behind_m=215)
        self.assertEqual(request_phase(geometry,'maintain')['phase'],'maintain')
        self.assertEqual(request_phase(geometry,'recover')['phase'],'recover')
        geometry.current.update(distance_m=99,sector_error_m=1)
        self.assertEqual(request_phase(geometry,'maintain')['phase'],'recover')
        geometry.relative=(150,-1)
        geometry.current=geometry.position(geometry.relative,geometry.course)
        self.assertTrue(geometry.current['in_sector'])
        self.assertEqual(request_phase(geometry,'recover')['phase'],'recover')
        geometry.relative=(0,-150)
        geometry.current=geometry.position(geometry.relative,geometry.course)
        self.assertEqual(request_phase(geometry,'recover')['phase'],'maintain')

    def test_phase_uses_current_snapshot_and_reset_clears_hysteresis(self):
        task,records=self.wire()
        adapter,obs=self.adapted(task,records)
        self.assertEqual(obs['task']['following_request']['phase'],'maintain')
        # The target reverses while range is unchanged: this is a loss.
        task['unity_only'].update(tick=50,decision_id='turn')
        records[1]['targetCourse']=180
        obs=adapter.observation(dict(task,situation=records),TASK)['observation']
        self.assertEqual(obs['task']['following_request']['phase'],'recover')
        self.assertEqual(obs['task']['following_request']['reason'],'ahead')
        p=build_payload(obs)
        self.assertEqual(len(p['questions']['action']['criteria']),128)
        self.assertLess(len(json.dumps(p).encode()),65000)
        # 15 m from the outer boundary is within leave but outside enter.
        records[1].update(targetCourse=0,longitude=geo(0,215)[0],latitude=geo(0,215)[1])
        adapter.task_state['following_phase']='maintain'
        task['unity_only'].update(tick=100,decision_id='near')
        obs=adapter.observation(dict(task,situation=records),TASK)['observation']
        self.assertEqual(obs['task']['following_request']['phase'],'maintain')
        task['id']='fresh-run'
        task['unity_only'].update(tick=0,decision_id='reset')
        adapter.add_task(task,TASK)
        obs=adapter.observation(dict(task,situation=records),TASK)['observation']
        self.assertEqual(obs['task']['following_request']['phase'],'recover')

    def test_region_forecast_uses_the_same_observed_turn_as_safety(self):
        from geometry.following import FollowingGeometry
        _,obs = self.adapted()
        straight = FollowingGeometry(obs)
        obs['task']['target_motion'] = dict(yaw_deg_s=12,acceleration_mps2=0)
        turning = FollowingGeometry(obs)
        straight.relative=turning.relative=(140,-30)
        motion = predict_ground_motion(10*KNOT_MPS,10*KNOT_MPS,0,obs['execution'],12,dt=.1,route_details=False)
        a,b = (g.forecast(motion,12,.1) for g in (straight,turning))
        displacement,course,_=turning.target_at(12)
        q=tuple(turning.relative[j]+motion['endpoint'][j]-displacement[j] for j in (0,1))
        from geometry.following import sector_facts
        expected=sector_facts(q,course,turning.inner,turning.outer,90-turning.angle_margin)
        self.assertAlmostEqual(b['region_error_m'],expected['sector_error_m'])
        # Maintaining the V4 point still does not chase a rotating rear point.
        self.assertAlmostEqual(a['tracking_error_m'],b['tracking_error_m'])
        self.assertEqual(turning.target_at(12)[1],144)

    def test_stop_is_separate_and_keeps_current_motion_safety_and_limit(self):
        task, records = self.wire()
        task['params']['maxSpeed'] = 18
        adapter, _ = self.adapted(task, records)
        stop = copy.deepcopy(task)
        stop.pop('params')
        stop['unity_only'].update(decision_id='stop', tick=50, situation_timestamp=124)
        adapter.add_task(stop, STOP)
        for record in records:
            record = dict(record, timestamp=124)
            if record['platformId'] == 3001:
                record.update(targetSpeed=0, targetCourse=None)
            adapter.add_situation(record)
        _, request, error = adapter.ready()[0]
        self.assertIsNone(error)
        self.assertTrue(valid_request(request))
        obs = request['observation']
        self.assertEqual(obs['task']['reference_course_deg'], 0)
        self.assertEqual(obs['action_space']['speeds_knots'][-1], 18)
        payload = build_payload(obs)
        self.assertEqual(payload['state']['task']['mode'], 'immediate_stop')
        self.assertNotIn('relative', payload['state'])
        self.assertIn('IMMEDIATE STOP', payload['questions']['action']['instructions'])
        for name in payload['questions']['action']['criteria']:
            facts = following_facts(payload, name)
            self.assertNotIn('sector_error_m', facts)
            self.assertIn('target_hull_clearance_m', facts)
        self.assertEqual(len(payload['questions']['action']['criteria']), 128)

    def test_stationary_start_needs_reference_and_stop_freezes_last_course(self):
        task, records = self.wire()
        records[1].update(targetSpeed=0, targetCourse=None)
        task['unity_only']['target_reference_course_deg'] = 270
        _, obs = self.adapted(task, records)
        self.assertEqual(obs['task']['reference_course_deg'], 270)
        task['unity_only'].pop('target_reference_course_deg')
        adapter = UnitySnapshotAdapter()
        with self.assertRaisesRegex(ValueError, 'reference direction'):
            adapter.observation(dict(task, situation=records), TASK)

    def test_observed_turn_wraps_angles_uses_ticks_and_resets_between_tasks(self):
        task, records = self.wire()
        records[1]['targetCourse'] = 359
        adapter, _ = self.adapted(task, records)
        task['unity_only'].update(tick=50, decision_id='second')
        records[1]['targetCourse'] = 1
        second = adapter.observation(dict(task, situation=records), TASK)['observation']
        self.assertAlmostEqual(second['task']['target_motion']['yaw_deg_s'], 2/1.7, places=5)
        task['unity_only'].update(tick=100, decision_id='third')
        records[1].update(targetCourse=None, targetSpeed=0)
        stopped = adapter.observation(dict(task, situation=records), TASK)['observation']
        self.assertEqual(stopped['task']['reference_course_deg'], 1)
        self.assertEqual(stopped['task']['target_motion']['yaw_deg_s'], 0)
        task['id'] = 'new-run'
        task['unity_only'].update(tick=0, decision_id='new')
        adapter.add_task(task, TASK)
        reset = adapter.observation(dict(task, situation=records), TASK)['observation']
        self.assertEqual(reset['task']['target_motion']['acceleration_mps2'], 0)

    def test_coordinated_turn_forecast_and_swept_collision_between_samples(self):
        from geometry.following import FollowingGeometry
        _, obs = self.adapted()
        obs['task']['target_motion'] = dict(yaw_deg_s=10, acceleration_mps2=0)
        geometry = FollowingGeometry(obs)
        displacement, course, speed = geometry.target_at(9)
        radius = 10*KNOT_MPS/math.radians(10)
        self.assertAlmostEqual(displacement[0], radius, delta=.001)
        self.assertAlmostEqual(displacement[1], radius, delta=.001)
        self.assertEqual(course, 90)
        geometry.relative = (-10, 0)
        geometry.speed = geometry.yaw = geometry.acceleration = 0
        geometry._target_cache.clear()
        facts = geometry.forecast(dict(path=[[0, 0], [20, 0]], endpoint=[20, 0], end_speed=20, end_course_deg=90), 1, 1)
        self.assertEqual(facts['minimum_m'], 0)

    def test_status_keeps_formal_distance_semantics_even_when_ahead(self):
        task, records = self.wire()
        records[1]['targetCourse'] = 180
        _, obs = self.adapted(task, records)
        content = status_content(task, 1, 'following', {}, observation=obs)
        params = content['params']
        self.assertEqual(content['code'], 'followMode')
        self.assertEqual(params['targetPlatCode'], 3001)
        self.assertEqual(params['inRange'], 1)
        self.assertAlmostEqual(params['relativeBearingDeg'], 0)
        self.assertAlmostEqual(params['distanceOutM'], 0)
        from geometry.following import FollowingGeometry
        self.assertFalse(FollowingGeometry(obs).current['in_sector'])
        final = status_content(dict(task, params=params), 2, 'completed', {})
        self.assertEqual(final['params']['relativeDistanceM'], params['relativeDistanceM'])
        self.assertEqual(final['params']['inRange'], 1)
        assembler = Reassembler()
        for data in reversed(json_packets(content, STATUS, stamp=123)):
            decoded = assembler.add(data, ('127.0.0.1', 5005))
        self.assertEqual(decoded, content)

    def test_old_distance_code_is_not_silently_aliased(self):
        for code in ('targetFollowing', 'unknown'):
            task, _ = self.wire()
            task['code'] = code
            with self.assertRaises(ValueError): UnitySnapshotAdapter().add_task(task, TASK)
            content = status_content(task, 3, 'unsupported task', {})
            self.assertEqual(content['code'], code)
            self.assertEqual(content['status'], 3)

    def test_task_speed_cap_and_parameter_boundaries(self):
        for cap in (0, 18, 80):
            task, records = self.wire()
            task['params']['maxSpeed'] = cap
            _, obs = self.adapted(task, records)
            self.assertEqual(obs['action_space']['speeds_knots'][-1], min(50, cap) if cap else 50)
        for field, value in [('followDistanceMinM', float('nan')), ('followDistanceMaxM', 50),
                             ('targetPlatCode', 1001), ('targetPlatCode', 9999), ('maxSpeed', -1)]:
            task, _ = self.wire()
            task['params'][field] = value
            with self.assertRaises(ValueError): UnitySnapshotAdapter().add_task(task, TASK)
        task, records = self.wire()
        task['params']['followDistanceMinM'] = task['params']['followDistanceMaxM'] = 150
        self.adapted(task, records)
        task['params']['followDistanceMinM'] = 0
        self.adapted(task, records)

    def test_motion_sweep_detects_crossing_between_samples(self):
        motion = {'path': [[0, 0], [20, 0]], 'sample_dt_s': 1}
        self.assertEqual(moving_range(motion, [10, 0], [0, 0], 1), (0, 10, [-10, 0]))
        self.assertEqual(moving_range(motion, [100, 0], [20, 0], 1)[:2], (100, 100))



if __name__ == "__main__":
    unittest.main()
