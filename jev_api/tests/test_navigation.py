import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choice_control import build_payload, decode_action, ACTION_CHOICES
from tools.waypoint_geometry import (
    xy,
    EARTH_RADIUS,
    KNOT_MPS,
    WaypointGeometry,
    circle_interval,
)
from boat_client import valid_request
from tools.navigation_motion import predict_ground_motion

ROOT = Path(__file__).resolve().parents[2]
ORIGIN = [120.5, 30.2]


def geo(x, y):
    return [
        ORIGIN[0] + math.degrees(x / EARTH_RADIUS / math.cos(math.radians(ORIGIN[1]))),
        ORIGIN[1] + math.degrees(y / EARTH_RADIUS),
    ]


def observation(level=0):
    p = (
        ROOT
        / "Assets/StreamingAssets"
        / ("fixed_route.json" if level == 0 else f"Maps/level_{level}.json")
    )
    route = json.loads(p.read_text(encoding="utf-8-sig"))
    return {
        "model_speed_unit": "mps",
        "planning": {
            "action_duration_s": 0.4,
            "simulation_paused_while_deciding": True,
        },
        "action_space": {
            "speeds_knots": [
                v / KNOT_MPS for v in (0, 0.25, 0.5, 1, 1.75, 2.5, 3.75, 5)
            ],
            "courses_deg": list(range(0, 360, 45)),
        },
        "boat": {
            "longitude": ORIGIN[0],
            "latitude": ORIGIN[1],
            "targetSpeed": 0,
            "targetCourse": None,
        },
        "execution": {"speed_transition_time_s": 0.5, "course_transition_time_s": 0.5},
        "task": {
            "planWaypointNum": len(route["waypoints"]) - 1,
            "next_waypoint_index": 0,
            "routeSpeed_unit": "m/s",
            "planWaypointList": [
                dict(
                    zip(("routeLon", "routeLat"), geo(p["x"], p["y"])),
                    routeSpeed=p["routeSpeed"],
                )
                for p in route["waypoints"][1:]
            ],
            "arrival_radius_m": route["arrival_radius_m"],
            "stop_speed_mps": route["stop_speed_mps"],
        },
    }


def packet(obs=None):
    return dict(
        type="decision_request",
        session_id="s",
        episode_id="e",
        run_id="r",
        decision_id="d",
        tick=0,
        observation=obs or observation(),
    )


class NavigationTests(unittest.TestCase):
    def test_geographic_roundtrip(self):
        self.assertAlmostEqual(xy(geo(10, 20), ORIGIN)[0], 10, places=5)
        self.assertAlmostEqual(xy(geo(10, 20), ORIGIN)[1], 20, places=5)

    def test_global_cardinal_motion(self):
        for course, expected in [
            (0, (0, 2)),
            (90, (2, 0)),
            (180, (0, -2)),
            (270, (-2, 0)),
        ]:
            m = predict_ground_motion(2, course, 2, course, 0, 0, 1)
            for a, b in zip(m["endpoint"], expected):
                self.assertAlmostEqual(a, b, places=5)

    def test_negative_speed_does_not_reverse_reported_ground_course(self):
        m = predict_ground_motion(-2, 90, 2, 90, 0, 0, 1)
        self.assertAlmostEqual(m["endpoint"][0], 2)

    def test_unknown_course_at_rest_and_forecast_is_explicit(self):
        p = build_payload(observation())
        self.assertIsNone(
            p["state"]["boat"]["movement_course_deg_clockwise_from_north"]
        )
        self.assertIn("Unknown startup", p["state"]["execution"]["forecast"])
        self.assertTrue(valid_request(packet()))

    def test_no_hidden_bow_or_reference_path_is_required(self):
        p = build_payload(observation())
        encoded = json.dumps(p)
        for key in (
            "reference_lon_lat",
            "ordered_route_ahead",
            "cross_track",
            "end_yaw",
            "preferred_tracking",
            "bow direction NOW",
        ):
            self.assertNotIn(key, encoded)
        self.assertNotIn("route", p["state"])
        self.assertAlmostEqual(p["state"]["target"]["distance_m"], 10, places=3)

    def test_all_choices_decode_global_physical_commands(self):
        obs = observation()
        self.assertEqual(len(ACTION_CHOICES), 64)
        for name, (i, j) in ACTION_CHOICES.items():
            result = decode_action(
                {"answers": {"action": {"type": "choice", "choice": name}}}, obs
            )
            self.assertEqual(
                result,
                {"speed": obs["action_space"]["speeds_knots"][i], "course": j * 45},
            )
        with self.assertRaises(ValueError):
            decode_action(
                {"answers": {"action": {"type": "choice", "choice": "bad"}}}, obs
            )

    def test_protocol_rejects_unavailable_moving_course_and_invalid_data(self):
        for field, value in [
            ("longitude", float("nan")),
            ("latitude", 91),
            ("targetCourse", 360),
            ("targetSpeed", float("inf")),
        ]:
            p = packet()
            p["observation"]["boat"][field] = value
            self.assertFalse(valid_request(p))
        p = packet()
        del p["observation"]["boat"]["targetSpeed"]
        self.assertFalse(valid_request(p))
        p = packet()
        p["observation"]["boat"]["targetSpeed"] = 1
        self.assertFalse(valid_request(p))
        p = packet()
        p["observation"]["task"]["planWaypointList"][1]["routeSpeed"] = -1
        self.assertFalse(valid_request(p))

    def test_waypoint_speed_is_ceiling_and_slow_candidate_is_allowed(self):
        p = build_payload(observation())
        self.assertEqual(p["state"]["target"]["passing_speed_ceiling_mps"], 2.5)
        self.assertIn("only a ceiling", p["state"]["task"]["background"])
        self.assertEqual(len(p["questions"]["action"]["criteria"]), 64)

    def test_swept_circle_and_interpolated_speed(self):
        c = WaypointGeometry(observation())
        c.waypoints = [(-10, 0), (0, 0)]
        c.next_index = 1
        c.radius = 1
        facts = c.motion_facts(
            {"path": [[-2, 0], [2, 0]], "path_speeds": [4, 0], "endpoint": [2, 0]}
        )
        self.assertTrue(facts["enters_checkpoint"])
        self.assertAlmostEqual(facts["max_speed_in_circle_mps"], 3)
        self.assertIsNone(circle_interval((-2, 2), (2, 2), (0, 0), 1))
        near = {
            "path": [[-2, 0], [-1.1, 0]],
            "path_speeds": [2, 2],
            "endpoint": [-1.1, 0],
        }
        self.assertFalse(c.motion_facts(near)["enters_checkpoint"])
        self.assertTrue(c.motion_facts(near, radius=1.25)["enters_checkpoint"])
        self.assertEqual(
            c.radius, 1
        )  # Advisory margin never changes the physical arrival circle.

    def test_passed_circle_ceiling_is_visible_in_each_candidate(self):
        obs = observation(4)
        obs["task"]["next_waypoint_index"] = 1
        previous = obs["task"]["planWaypointList"][0]
        lon, lat = previous["routeLon"], previous["routeLat"]
        obs["boat"].update(
            longitude=lon, latitude=lat, targetSpeed=1 / KNOT_MPS, targetCourse=90
        )
        payload = build_payload(obs)
        self.assertEqual(
            payload["state"]["recently_passed_circle"]["speed_ceiling_mps"],
            previous["routeSpeed"],
        )
        criteria = payload["questions"]["action"]["criteria"]
        self.assertIn("EXCEEDS the previous ceiling", criteria["speed_7__east"])
        self.assertIn("does not exceed the previous ceiling", criteria["speed_3__east"])
        self.assertTrue(all(isinstance(v, str) for v in criteria.values()))

    def test_final_parking_stage(self):
        obs = observation()
        obs["task"]["next_waypoint_index"] = obs["task"]["planWaypointNum"] - 1
        end = obs["task"]["planWaypointList"][-1]
        obs["boat"].update(longitude=end["routeLon"], latitude=end["routeLat"])
        p = build_payload(obs)
        self.assertEqual(p["state"]["task"]["stage"], "final_parking")
        self.assertTrue(p["state"]["target"]["inside_arrival_circle"])

    def test_prediction_path_speed_alignment(self):
        m = predict_ground_motion(3, 90, 0, 0, 0.13, 0.17, 0.4)
        self.assertEqual(len(m["path"]), len(m["path_speeds"]))
        self.assertAlmostEqual(m["end_speed"], 0)

    def test_all_maps_and_udp_budget(self):
        for level in range(5):
            p = packet(observation(level))
            self.assertTrue(valid_request(p))
            self.assertLess(len(json.dumps(p).encode()), 65000)
            build_payload(p["observation"])

    def test_course_change_wraps_and_settles(self):
        motion = predict_ground_motion(2, 359, 2, 1, 0, 0.5, 0.5)
        self.assertGreater(motion["endpoint"][1], 0.99)
        self.assertLess(abs(motion["endpoint"][0]), 0.01)
        # A settled northward course continues straight after the transition.
        longer = predict_ground_motion(2, 90, 2, 0, 0, 0.5, 0.8)
        settled = predict_ground_motion(2, 90, 2, 0, 0, 0.5, 0.5)
        self.assertAlmostEqual(longer["endpoint"][0], settled["endpoint"][0], places=3)
        self.assertAlmostEqual(
            longer["endpoint"][1] - settled["endpoint"][1], 0.6, places=3
        )

    def test_turning_displacement_is_curved(self):
        motion = predict_ground_motion(2, 90, 2, 0, 0, 0.5, 0.5)
        self.assertGreater(motion["endpoint"][0], 0.4)
        self.assertGreater(motion["endpoint"][1], 0.4)
        self.assertLess(motion["endpoint"][0], 1)

    def test_fixed_speed_arrival_time_and_hold(self):
        a = predict_ground_motion(0, None, 2, 90, 0.5, 0, 0.25)
        self.assertAlmostEqual(a["end_speed"], 1)
        self.assertAlmostEqual(a["endpoint"][0], 0.125)
        b = predict_ground_motion(0, None, 2, 90, 0.5, 0, 1)
        self.assertAlmostEqual(b["end_speed"], 2)
        self.assertAlmostEqual(b["endpoint"][0], 1.5)

    def test_braking_ignores_the_requested_course(self):
        a = predict_ground_motion(2, 90, 0, 90, 0.5, 0.5, 0.4)
        b = predict_ground_motion(2, 90, 0, 270, 0.5, 0.5, 0.4)
        self.assertEqual(a, b)
        self.assertGreater(a["endpoint"][0], 0)
        self.assertAlmostEqual(a["endpoint"][1], 0)


if __name__ == "__main__":
    unittest.main()
