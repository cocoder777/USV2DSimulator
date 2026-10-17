"""Protocol checks; no network requests or API credentials are used."""
import copy
import io
import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import boat_client as client


def observation(count=1):
    return {
        "coordinates": "Global XY metres; counterclockwise degrees from east.",
        "planning": {"action_count": count, "action_duration_s": 0.4,
                     "simulation_paused_while_deciding": True},
        "actions": {"acceleration_fractions": [-1, -2/3, -1/3, 0, .25, .5, .75, 1],
                    "turn_fractions": [-1, -.5, -.25, 0, .25, .5, 1]},
        "boat": {"position_xy_m": [0, 0], "heading_deg": 0, "speed_mps": 1,
                 "yaw_rate_deg_s": 8, "actuator_acceleration_mps2": .2,
                 "actuator_angular_acceleration_deg_s2": 6},
        "target": {"position_xy_m": [8, 3], "arrival_radius_m": 1,
                   "requires_stop": True, "stop_speed_mps": .1},
        "dynamics": {"max_acceleration": 1.5, "max_angular_acceleration_deg": 60},
        "motion": "First-order actuator lag and damping.",
    }


def response(count=1, acceleration=4.2, turn=2.1):
    return {"answers": {f"{axis}_{i}": {"type": "score", "score": score}
                        for i in range(count)
                        for axis, score in (("acceleration", acceleration), ("turn", turn))}}


class DecisionProtocolTests(unittest.TestCase):
    def test_single_and_multiple_action_requests(self):
        for count in (1, 3, 10):
            with self.subTest(count=count):
                obs = observation(count)
                original = copy.deepcopy(obs)
                payload = client.build_payload(obs)
                self.assertEqual(obs, original)
                self.assertEqual(len(payload["questions"]), 2 * count)
                for i in range(count):
                    for axis, levels in (("acceleration", 8), ("turn", 7)):
                        question = payload["questions"][f"{axis}_{i}"]
                        self.assertEqual(question["type"], "score")
                        self.assertEqual(len(question["criteria"]), levels)
                        self.assertIsInstance(question["criteria"], list)
                        if count == 1:
                            self.assertIn("next 0.4 seconds", question["instructions"])
                            self.assertNotIn("time zero", question["instructions"])
                        else:
                            self.assertIn(f"action {i + 1} of {count}", question["instructions"])
                            self.assertIn("measured at time zero", question["instructions"])
                            self.assertIn("without intermediate feedback", question["instructions"])
                if count == 3:
                    self.assertIn("[0.4, 0.8)", payload["questions"]["turn_1"]["instructions"])
                self.assertIn("-1.5 m/s^2", payload["questions"]["acceleration_0"]["criteria"][0])
                self.assertIn("60 deg/s^2", payload["questions"]["turn_0"]["criteria"][-1])

    def test_model_state_is_whitelisted_and_detached(self):
        obs = observation()
        obs["transport_only"] = {"must_not_reach_model": True}
        payload = client.build_payload(obs)
        self.assertEqual(set(payload["state"]), set(client.MODEL_STATE_FIELDS) | {"task"})
        self.assertNotIn("must_not_reach_model", client.dumps(payload))
        payload["state"]["boat"]["position_xy_m"][0] = 99
        self.assertEqual(obs["boat"]["position_xy_m"], [0, 0])

    def test_axes_have_same_template_without_extra_turn_advice(self):
        for count in (1, 3):
            a = client.make_instructions("acceleration", 0, count, .4)
            t = client.make_instructions("turn", 0, count, .4)
            self.assertEqual(a.replace("longitudinal", "angular"), t)

    def test_scores_round_to_existing_discrete_indices(self):
        self.assertEqual(client.decode_actions(response(3), 3),
                         [{"acceleration_index": 4, "turn_index": 2}] * 3)
        for a, t, expected in ((0, 0, (0, 0)), (7, 6, (7, 6)), (4.5, 2.5, (5, 3))):
            action = client.decode_actions(response(acceleration=a, turn=t), 1)[0]
            self.assertEqual((action["acceleration_index"], action["turn_index"]), expected)

    def test_invalid_scores_and_answer_shapes_fail(self):
        for axis, maximum in (("acceleration", 7), ("turn", 6)):
            for score in (-.01, maximum + .01, math.nan, math.inf, True, "3", None):
                raw = response()
                raw["answers"][axis + "_0"]["score"] = score
                with self.subTest(axis=axis, score=score), self.assertRaises(ValueError):
                    client.decode_actions(raw, 1)
        for raw in ({"answers": {}}, {"answers": []},
                    {"answers": {"wrong_a": {}, "wrong_t": {}}}):
            with self.assertRaises(ValueError):
                client.decode_actions(raw, 1)
        raw = response()
        raw["answers"]["turn_0"] = {"type": "choice", "choice": "2"}
        with self.assertRaises(ValueError):
            client.decode_actions(raw, 1)

    def test_decide_preserves_wire_contract_and_logs_actual_payload(self):
        packet = {"type": "decision_request", "version": 4, "tick": 12,
                  "session_id": "s", "episode_id": "e", "run_id": "r", "decision_id": "d",
                  "observation": observation(3)}
        self.assertTrue(client.valid_request(packet))
        log = io.StringIO()
        settings = SimpleNamespace(model="jev-latest", endpoint="https://example.invalid")
        with patch.object(client, "call_api", return_value=response(3)) as api, patch("builtins.print"):
            result = client.decide(packet, settings, "test-key", log)
        self.assertEqual(result["type"], "decision_result")
        self.assertEqual(result["action_duration_s"], .4)
        self.assertEqual(len(result["actions"]), 3)
        for field in client.IDENTITY_FIELDS:
            self.assertEqual(result[field], packet[field])
        record = json.loads(log.getvalue())
        self.assertEqual(record["payload"], api.call_args.args[0])
        self.assertEqual(record["result"], result)
        self.assertNotIn("test-key", log.getvalue())


if __name__ == "__main__":
    unittest.main()
