import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choice_control import build_payload, decode_action, ACTION_CHOICES, speed_response
from route_context import RouteContext, xy, EARTH_RADIUS
from boat_client import valid_request
from decision_worker import PROTOCOL_VERSION
from tools.navigation_motion import turn_response, predict_motion, delta_angle

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_jev import trajectory_metrics

ORIGIN = [120.5, 30.2]


def geo(x, y):
    return [
        round(
            ORIGIN[0]
            + math.degrees(x / EARTH_RADIUS / math.cos(math.radians(ORIGIN[1]))),
            8,
        ),
        round(ORIGIN[1] + math.degrees(y / EARTH_RADIUS), 8),
    ]


def observation(points=None, waypoints=None, next_index=1):
    if points is None:
        route = json.loads(
            (
                Path(__file__).resolve().parents[2]
                / "Assets/StreamingAssets/fixed_route.json"
            ).read_text()
        )
        points = [(p["x"], p["y"]) for p in route["path"]["samples"]]
        waypoints = [(p["x"], p["y"]) for p in route["waypoints"]]
    return {
        "planning": {
            "action_duration_s": 0.4,
            "simulation_paused_while_deciding": True,
        },
        "action_space": {
            "speeds_mps": [-1, -0.5, 0, 0.5, 1.25, 2.5, 3.75, 5],
            "heading_offsets_deg": [0, 45, 90, 135, 180, -135, -90, -45],
        },
        "boat": {
            "lon_lat": geo(*points[0]),
            "heading_deg": 0,
            "speed_mps": 0,
            "yaw_rate_deg_s": 0,
        },
        "response": {
            "speed_transition_time_s": 0,
            "active_target_speed_mps": 0,
            "remaining_transition_s": 0,
        },
        "heading_response": {
            "transition_time_s": 0,
            "active_target_offset_deg": 0,
            "remaining_transition_s": 0,
        },
        "fixed_dt_s": 0.02,
        "task": {
            "route_id": "test",
            "reference_lon_lat": [geo(*p) for p in points],
            "origin_lon_lat": ORIGIN,
            "planWaypointList": [geo(*p) for p in waypoints],
            "planWaypointNum": len(waypoints),
            "next_waypoint_index": next_index,
            "currentRoute": next_index - 1,
            "arrival_radius_m": 1,
            "stop_speed_mps": 0.1,
        },
    }


class NavigationTests(unittest.TestCase):
    def test_hairpin_metrics_do_not_report_approach_as_missed(self):
        route_points = [(0, 0), (4.9, -0.1), (5, 0), (4.9, 0.1), (0, 1)]
        samples = [
            (0, 0),
            (1, 0),
            (2, 0),
            (3, 0),
            (4, 0),
            (5, 0),
            (4, 0.2),
            (2, 0.6),
            (0, 1),
        ]
        document = {
            "route": {
                "path": {
                    "samples": [dict(x=x, y=y) for x, y in route_points],
                    "waypoint_sample_indices": [0, 2, 4],
                },
                "waypoints": [
                    dict(id=str(i), x=x, y=y)
                    for i, (x, y) in enumerate(
                        [route_points[0], route_points[2], route_points[4]]
                    )
                ],
                "arrival_radius_m": 0.25,
                "stop_speed_mps": 0.1,
            },
            "samples": [
                dict(x=x, y=y, speed=0 if i == len(samples) - 1 else 1)
                for i, (x, y) in enumerate(samples)
            ],
            "simulation_time": 8,
            "passed_count": 3,
            "final_speed_mps": 0,
        }
        self.assertEqual(trajectory_metrics(document)["missed_points"], [])

    def test_metrics_still_report_gate_overshoot_before_recovery(self):
        points = [(0, 0), (2, 0), (5, 0)]
        samples = [(0, 0), (1, 0.75), (2, 0.75), (3, 0.75), (2, 0), (5, 0)]
        document = {
            "route": {
                "path": {
                    "samples": [dict(x=x, y=y) for x, y in points],
                    "waypoint_sample_indices": [0, 1, 2],
                },
                "waypoints": [
                    dict(id=str(i), x=x, y=y) for i, (x, y) in enumerate(points)
                ],
                "arrival_radius_m": 0.5,
                "stop_speed_mps": 0.1,
            },
            "samples": [
                dict(x=x, y=y, speed=0 if i == len(samples) - 1 else 1)
                for i, (x, y) in enumerate(samples)
            ],
            "simulation_time": 5,
            "passed_count": 3,
            "final_speed_mps": 0,
        }
        self.assertEqual(trajectory_metrics(document)["missed_points"], ["1"])

    def test_heading_and_yaw_are_continuous(self):
        self.assertEqual(turn_response(90, 20, 0.5, 0), (0, 20))
        self.assertEqual(turn_response(90, 20, 0.5, 0.5), (90, 0))
        self.assertEqual(turn_response(90, 20, 0.5, 0.8), (90, 0))
        self.assertEqual(delta_angle(1 - 359), 2)
        self.assertEqual(delta_angle(359 - 1), -2)
        angle, yaw = turn_response(-135, 500, 0.5, 0.02)
        end, yaw_end = turn_response(-135 - angle, yaw, 0.48, 0.2)
        expected, expected_yaw = turn_response(-135, 500, 0.5, 0.22)
        self.assertAlmostEqual(angle + end, expected)
        self.assertAlmostEqual(yaw_end, expected_yaw)

    def test_turning_displacement_is_curved(self):
        motion = predict_motion(2, 2, 0, 90, 0, 0.5, 0.5, 0.02)
        self.assertGreater(motion["endpoint"][0], 0.4)
        self.assertGreater(motion["endpoint"][1], 0.4)
        self.assertLess(motion["endpoint"][0], 1)
        self.assertEqual(motion["end_heading"], 90)
        self.assertEqual(motion["end_yaw"], 0)

    def test_heading_information_and_prompt(self):
        obs = observation()
        obs["heading_response"]["transition_time_s"] = 0.7
        obs["boat"]["yaw_rate_deg_s"] = 20
        p = build_payload(obs)
        self.assertIn("turning left at 20.000 deg/s", p["state"]["boat"]["rotation"])
        self.assertIn("0.7 s", p["state"]["execution"]["heading_rule"])
        self.assertIn("turns smoothly", p["state"]["execution"]["heading_rule"])
        self.assertNotIn(
            "heading changes INSTANTLY", p["questions"]["action"]["instructions"]
        )

    def test_geographic_roundtrip(self):
        for p in [(0, 0), (10, 5), (-200, 400)]:
            q = xy(geo(*p), ORIGIN)
            self.assertLess(math.dist(p, q), 0.002)

    def test_local_left_and_right(self):
        obs = observation([(0, 0), (0, 10)], [(0, 0), (0, 10)])
        self.assertEqual(
            RouteContext().prepare(obs)["target"]["forward_left_m"], [0, 10]
        )
        self.assertIn(
            "10.000 m left", build_payload(obs)["state"]["target"]["position"]
        )
        obs["boat"]["heading_deg"] = 180
        self.assertEqual(
            RouteContext().prepare(obs)["target"]["forward_left_m"], [0, -10]
        )
        self.assertIn(
            "10.000 m right", build_payload(obs)["state"]["target"]["position"]
        )

    def test_all_choices_decode_and_negative_motion(self):
        payload = build_payload(observation())
        self.assertEqual(len(payload["questions"]["action"]["criteria"]), 64)
        for name, action in ACTION_CHOICES.items():
            self.assertEqual(
                decode_action(
                    {"answers": {"action": {"type": "choice", "choice": name}}}
                ),
                action,
            )
        self.assertIn(
            "0.400 m right",
            payload["questions"]["action"]["criteria"]["reverse_full__left"],
        )
        with self.assertRaises(ValueError):
            decode_action(
                {
                    "answers": {
                        "action": {"type": "choice", "choice": "old_acceleration"}
                    }
                }
            )

    def test_minimal_model_input(self):
        state = build_payload(observation())["state"]
        self.assertEqual(set(state), {"task", "route", "target", "execution"})
        obs = observation(next_index=4)
        self.assertEqual(build_payload(obs)["state"]["task"]["stage"], "final_approach")

    def test_crossing_stays_on_active_leg(self):
        p = [(0, 0), (5, 5), (10, 10), (0, 10), (5, 5), (10, 0)]
        obs = observation(p, [p[0], p[2], p[3], p[-1]])
        obs["boat"]["lon_lat"] = geo(5, 5)
        ctx = RouteContext()
        ctx.prepare(obs, "a")
        self.assertLess(ctx.progress, 10)
        obs["task"]["next_waypoint_index"] = 3
        ctx.prepare(obs, "b")
        self.assertGreater(ctx.progress, 20)
        # Explicit run identity resets progress even with identical route data.
        obs["task"]["next_waypoint_index"] = 1
        obs["boat"]["lon_lat"] = geo(0, 0)
        ctx.prepare(obs, "c")
        self.assertLess(ctx.progress, 0.01)

    def test_preview_stops_at_pending_gate_and_labels_following_leg(self):
        points = [(0, 0), (5, 0), (5, 1), (0, 1)]
        obs = observation(points, points)
        obs["boat"]["lon_lat"] = geo(4, 0)
        route = RouteContext().prepare(obs)["route"]
        self.assertTrue(
            all(abs(p[1]) < 0.002 for p in route["ahead_points_forward_left_m"])
        )
        self.assertGreater(route["after_checkpoint_forward_left_m"][0][1], 0.9)

    def test_future_parallel_leg_does_not_hide_current_tracking_error(self):
        points = [(0, 0), (5, 0), (5, 1), (0, 1)]
        ctx = RouteContext()
        ctx.prepare(observation(points, points))
        facts = ctx.motion_facts([[0, 0], [0, 1], [2, 1]])
        self.assertAlmostEqual(facts["route_error_end_m"], 1, places=2)
        self.assertAlmostEqual(facts["end_route_tangent_deg"], 0, places=2)
        self.assertFalse(facts["enters_checkpoint"])

    def test_forecast_can_follow_next_leg_after_entering_gate(self):
        points = [(0, 0), (5, 0), (5, 1), (0, 1)]
        ctx = RouteContext()
        ctx.prepare(observation(points, points))
        facts = ctx.motion_facts([[0, 0], [5, 0], [5, 1]])
        self.assertTrue(facts["enters_checkpoint"])
        self.assertLess(facts["route_error_end_m"], 0.002)

    def test_crossing_forecast_cannot_teleport_progress(self):
        points = [(0, 0), (3, 3), (0, 3), (3, 0), (6, 0)]
        ctx = RouteContext()
        obs = observation(points, [points[0], points[-1]])
        obs["boat"]["lon_lat"] = geo(1.4, 1.4)
        ctx.prepare(obs)
        facts = ctx.motion_facts([[0, 0], [0.2, 0]])
        self.assertLess(abs(facts["along_route_advance_m"]), 0.25)

    def test_protocol_version_and_shape(self):
        p = dict(
            type="decision_request",
            version=PROTOCOL_VERSION,
            session_id="s",
            episode_id="e",
            run_id="r",
            decision_id="d",
            tick=0,
            observation=observation(),
        )
        self.assertTrue(valid_request(p))
        p["version"] = 5
        self.assertFalse(valid_request(p))

    def test_speed_response_and_composition(self):
        # Reversal moves backward first, even though the new target is positive.
        distance, speed = speed_response(-1, 1, 0.5, 0.1)
        self.assertLess(distance, 0)
        self.assertLess(speed, 0)
        total = 0
        v = 3
        remaining = 0.5
        for _ in range(20):
            d, v = speed_response(v, 0, remaining, 0.02)
            total += d
            remaining = max(0, remaining - 0.02)
        expected_distance, expected_speed = speed_response(3, 0, 0.5, 0.4)
        self.assertAlmostEqual(total, expected_distance)
        self.assertAlmostEqual(v, expected_speed)
        self.assertEqual(speed_response(3, 0, 0, 0.4), (0, 0))

    def test_response_information_only_when_needed(self):
        obs = observation()
        obs["boat"]["speed_mps"] = 3
        self.assertNotIn("boat", build_payload(obs)["state"])
        obs["response"]["speed_transition_time_s"] = 0.5
        payload = build_payload(obs)
        self.assertIn(
            "moving forward at 3.000 m/s", payload["state"]["boat"]["translation"]
        )
        self.assertIn("0.5 s", payload["state"]["execution"]["speed_rule"])
        # Target zero is newly selected here, not an already completed zero target.
        obs["response"]["active_target_speed_mps"] = 3
        payload = build_payload(obs)
        stop = payload["questions"]["action"]["criteria"]["stop__ahead"]
        self.assertIn("0.720 m ahead", stop)
        self.assertIn("0.600 m/s", stop)

    def test_fixed_arrival_time_and_hold(self):
        for target in (1, 2, -1):
            self.assertAlmostEqual(speed_response(0, target, 0.5, 0.25)[1], target / 2)
            self.assertEqual(speed_response(0, target, 0.5, 0.5)[1], target)
            self.assertAlmostEqual(
                speed_response(0, target, 0.5, 0.8)[0], target * (0.25 + 0.3)
            )

    def test_same_target_continues_remaining_ramp(self):
        obs = observation()
        obs["boat"]["speed_mps"] = 1
        obs["response"] = {
            "speed_transition_time_s": 0.5,
            "active_target_speed_mps": 1.25,
            "remaining_transition_s": 0.1,
        }
        criterion = build_payload(obs)["questions"]["action"]["criteria"][
            "forward_quarter__ahead"
        ]
        self.assertIn("0.488 m ahead", criterion)
        self.assertIn("1.250 m/s", criterion)

    def test_tunable_parameters_are_not_fixed_in_prompt(self):
        obs = observation(next_index=4)
        obs["task"]["arrival_radius_m"] = 2.3
        obs["task"]["stop_speed_mps"] = 0.07
        obs["planning"]["action_duration_s"] = 0.8
        obs["response"]["speed_transition_time_s"] = 1.2
        obs["action_space"]["speeds_mps"] = [-2, -1, 0, 1, 2.5, 5, 7.5, 10]
        payload = build_payload(obs)
        self.assertEqual(payload["state"]["target"]["arrival_radius_m"], 2.3)
        self.assertEqual(payload["state"]["execution"]["duration_s"], 0.8)
        self.assertIn("1.2 s", payload["state"]["execution"]["speed_rule"])
        self.assertIn("0.07 m/s", payload["state"]["task"]["current_goal"])
        self.assertIn(
            "moving forward at 10.000 m/s",
            payload["questions"]["action"]["criteria"]["forward_full__ahead"],
        )
        self.assertNotIn("radius=1 m", payload["questions"]["action"]["instructions"])

    def test_final_arrival_is_explicit(self):
        obs = observation([(0, 0), (10, 0)], [(0, 0), (10, 0)])
        obs["boat"]["lon_lat"] = geo(8.9, 0)
        target = build_payload(obs)["state"]["target"]
        self.assertAlmostEqual(target["distance_m"], 1.1, places=2)
        self.assertFalse(target["inside_arrival_circle"])
        obs["boat"]["lon_lat"] = geo(9.1, 0)
        self.assertTrue(build_payload(obs)["state"]["target"]["inside_arrival_circle"])

    def test_candidate_route_facts_are_geometry_not_recommendations(self):
        ctx = RouteContext()
        ctx.prepare(observation())
        facts = ctx.motion_facts([[0, 0], [1, 0], [2, 0]])
        self.assertAlmostEqual(facts["route_error_max_m"], 0, places=3)
        self.assertAlmostEqual(facts["along_route_advance_m"], 2, places=2)
        self.assertFalse(facts["enters_checkpoint"])
        self.assertNotIn("score", facts)
        self.assertNotIn("recommended_action", facts)
        obs = observation([(0, 0), (0.05, 0)], [(0, 0), (0.05, 0)])
        obs["task"]["arrival_radius_m"] = 0.01
        ctx.prepare(obs)
        # A fast step may cross a small circle between its endpoints.
        self.assertTrue(ctx.motion_facts([[0, 0], [0.1, 0]])["enters_checkpoint"])

    def test_forecast_path_does_not_change_motion(self):
        args = (2, 3, 0.5, 45, 20, 0.5, 0.4, 0.02)
        plain = predict_motion(*args)
        traced = predict_motion(*args, include_path=True)
        for key in plain:
            self.assertEqual(plain[key], traced[key])
        self.assertGreater(len(traced["path"]), 10)

    def test_tracking_preference_is_separate_from_arrival_radius(self):
        obs = observation()
        obs["task"]["arrival_radius_m"] = 2.3
        payload = build_payload(obs, tracking_preference_m=0.12)
        self.assertEqual(payload["state"]["task"]["preferred_tracking_error_m"], 0.12)
        self.assertEqual(payload["state"]["target"]["arrival_radius_m"], 2.3)
        self.assertIn(
            "hypothetical",
            payload["questions"]["action"]["criteria"]["forward_full__ahead"],
        )
        with self.assertRaises(ValueError):
            build_payload(obs, tracking_preference_m=0)

    def test_task_background_and_final_parking_are_explicit(self):
        obs = observation()
        self.assertIn(
            "mandatory checkpoints", build_payload(obs)["state"]["task"]["background"]
        )
        self.assertEqual(
            build_payload(obs)["state"]["task"]["stage"], "checkpoint_tracking"
        )
        obs["task"]["next_waypoint_index"] = 4
        obs["boat"]["lon_lat"] = geo(39.5, 0)
        p = build_payload(obs)
        self.assertEqual(p["state"]["task"]["stage"], "final_parking")
        self.assertIn("0.1 m/s", p["state"]["task"]["current_goal"])
        self.assertTrue(p["state"]["target"]["inside_arrival_circle"])

    def test_all_maps_fit_udp_budget(self):
        root = Path(__file__).resolve().parents[2] / "Assets/StreamingAssets"
        for f in [root / "fixed_route.json", *(root / "Maps").glob("*.json")]:
            r = json.loads(f.read_text())
            obs = observation(
                [(p["x"], p["y"]) for p in r["path"]["samples"]],
                [(p["x"], p["y"]) for p in r["waypoints"]],
            )
            # Compact JSON matches Unity wire formatting; reserve space for envelope and metadata.
            self.assertLess(
                len(json.dumps(obs, separators=(",", ":")).encode()) + 1000,
                65000,
                f.name,
            )


if __name__ == "__main__":
    unittest.main()
