import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import classic_control as controller


def observation(**overrides):
    value = {
        "planning": {"action_duration_s": 0.4, "simulation_paused_while_deciding": True},
        "action_space": {
            "acceleration_fractions": [-1, -2 / 3, -1 / 3, 0, 0.25, 0.5, 0.75, 1],
            "turn_fractions": [-1, -0.5, -0.25, 0, 0.25, 0.5, 1],
        },
        "boat": {
            "position_xy_m": [0.0, 0.0],
            "heading_deg": 0.0,
            "speed_mps": 0.0,
            "yaw_rate_deg_s": 0.0,
        },
        "target": {
            "position_xy_m": [10.0, 0.0],
            "arrival_radius_m": 1.0,
            "requires_stop": False,
            "stop_speed_mps": 0.1,
        },
        "dynamics": {
            "max_forward_speed": 5.0,
            "max_reverse_speed": 1.0,
            "max_acceleration": 1.5,
            "max_angular_acceleration_deg": 60.0,
            "max_yaw_rate_deg": 45.0,
            "linear_damping": 0.2,
            "angular_damping": 1.2,
        },
        "motion": "test",
    }
    for key, patch in overrides.items():
        if isinstance(patch, dict):
            value[key].update(patch)
        else:
            value[key] = patch
    return value


class PredictionTests(unittest.TestCase):
    def test_prediction_matches_recorded_unity_step_result(self):
        predicted = controller.predict_action(observation(), 7, 3)
        self.assertAlmostEqual(predicted.x, 0.1228647, places=5)
        self.assertAlmostEqual(predicted.speed_mps, 0.577738, places=5)
        self.assertAlmostEqual(predicted.y, 0.0, places=7)
        self.assertAlmostEqual(predicted.heading_deg, 0.0, places=7)

    def test_prediction_uses_action_fractions_and_limits(self):
        predicted = controller.predict_action(observation(), 0, 3)
        self.assertAlmostEqual(predicted.x, -0.1228647, places=5)
        self.assertAlmostEqual(predicted.speed_mps, -0.577738, places=5)

    def test_prediction_rejects_non_integral_action_duration(self):
        obs = observation(planning={"action_duration_s": 0.41})
        with self.assertRaises(ValueError):
            controller.predict_action(obs, 7, 3)


class ActionMappingTests(unittest.TestCase):
    def test_nearest_action_uses_actual_dynamics_values(self):
        action = controller.nearest_action(observation(), 0.375, -15.0)
        self.assertEqual(action, {"acceleration_index": 4, "turn_index": 2})

    def test_nearest_action_is_clamped_to_action_limits(self):
        action = controller.nearest_action(observation(), 99.0, -999.0)
        self.assertEqual(action, {"acceleration_index": 7, "turn_index": 0})


class DiscreteControllerTests(unittest.TestCase):
    def test_pd_target_ahead_from_rest_accelerates_forward(self):
        action = controller.pd_action(observation())
        self.assertGreater(action["acceleration_index"], 3)
        self.assertEqual(action["turn_index"], 3)

    def test_pd_left_and_right_targets_turn_in_the_matching_direction(self):
        left = controller.pd_action(observation(target={"position_xy_m": [8.0, 5.0]}))
        right = controller.pd_action(observation(target={"position_xy_m": [8.0, -5.0]}))
        self.assertGreater(left["turn_index"], 3)
        self.assertLess(right["turn_index"], 3)

    def test_pd_reverse_speed_target_ahead_requests_positive_acceleration(self):
        obs = observation(boat={"speed_mps": -1.0}, target={"position_xy_m": [6.0, 0.0]})
        self.assertGreater(controller.pd_action(obs)["acceleration_index"], 3)

    def test_pd_final_target_inside_stop_distance_brakes(self):
        obs = observation(
            boat={"speed_mps": 2.0},
            target={"position_xy_m": [1.0, 0.0], "requires_stop": True, "stop_speed_mps": 0.1},
        )
        self.assertLessEqual(controller.pd_action(obs)["acceleration_index"], 2)

    def test_lookahead_turns_toward_far_left_and_right_targets(self):
        left = controller.lookahead_action(observation(target={"position_xy_m": [8.0, 5.0]}))
        right = controller.lookahead_action(observation(target={"position_xy_m": [8.0, -5.0]}))
        self.assertGreater(left["turn_index"], 3)
        self.assertLess(right["turn_index"], 3)

    def test_pd_and_lookahead_creep_into_final_circle_from_just_outside(self):
        obs = observation(
            boat={"position_xy_m": [4.967, 0.0], "speed_mps": 0.0},
            target={"position_xy_m": [6.0, 0.0], "requires_stop": True, "stop_speed_mps": 0.1},
        )
        self.assertGreater(controller.pd_action(obs)["acceleration_index"], 3)
        self.assertGreater(controller.lookahead_action(obs)["acceleration_index"], 3)

    def test_lookahead_is_deterministic_and_in_range(self):
        first = controller.lookahead_action(observation())
        second = controller.lookahead_action(observation())
        self.assertEqual(first, second)
        self.assertTrue(0 <= first["acceleration_index"] <= 7)
        self.assertTrue(0 <= first["turn_index"] <= 6)

    def test_lookahead_final_approach_predicts_lower_speed(self):
        obs = observation(
            boat={"speed_mps": 2.0},
            target={"position_xy_m": [1.0, 0.0], "requires_stop": True, "stop_speed_mps": 0.1},
        )
        action = controller.lookahead_action(obs)
        predicted = controller.predict_action(obs, action["acceleration_index"], action["turn_index"])
        self.assertLess(predicted.speed_mps, 2.0)
        self.assertLessEqual(action["acceleration_index"], 3)

    def test_choose_action_supports_both_methods(self):
        for method in ("pd", "lookahead"):
            action = controller.choose_action(observation(), method)
            self.assertEqual(set(action), {"acceleration_index", "turn_index"})


class WorkerProtocolTests(unittest.TestCase):
    def packet(self, **overrides):
        value = {
            "type": "decision_request",
            "version": 5,
            "session_id": "s",
            "episode_id": "e",
            "run_id": "r",
            "decision_id": "d",
            "tick": 12,
            "observation": observation(),
        }
        value.update(overrides)
        return value

    def test_valid_request_accepts_current_protocol(self):
        self.assertTrue(controller.valid_request(self.packet()))

    def test_valid_request_rejects_wrong_version_and_old_action_fields(self):
        self.assertFalse(controller.valid_request(self.packet(version=4)))
        packet = self.packet()
        packet["observation"]["planning"]["action_count"] = 2
        self.assertFalse(controller.valid_request(packet))

    def test_decision_result_preserves_identity_and_duration(self):
        result = controller.decision_result(self.packet(), "lookahead")
        self.assertEqual(result["type"], "decision_result")
        self.assertEqual(result["version"], 5)
        self.assertEqual(result["session_id"], "s")
        self.assertEqual(result["tick"], 12)
        self.assertEqual(result["action_duration_s"], 0.4)
        self.assertTrue(0 <= result["action"]["acceleration_index"] <= 7)
        self.assertTrue(0 <= result["action"]["turn_index"] <= 6)

    def test_decision_error_preserves_identity(self):
        result = controller.decision_error(self.packet(), ValueError("bad state"))
        self.assertEqual(result["type"], "decision_error")
        self.assertEqual(result["decision_id"], "d")
        self.assertIn("bad state", result["error"])


if __name__ == "__main__":
    unittest.main()
