"""立即停止任务：无航路点输入、原 Choice 解码和工作线程调用。"""

import copy
from types import SimpleNamespace
import unittest

from test_navigation import observation, packet
from boat_client import valid_request
from choice_control import ACTION_CHOICES, build_payload, build_stop_payload, decode_action
from decision_worker import DecisionScheduler


def stop_packet():
    request = packet(observation())
    request["type"] = "stop_request"
    request["observation"]["operation"] = "stop"
    request["observation"]["task"] = {
        "code": "trackingMode", "stop_speed_mps": 0.000001,
    }
    request["observation"]["boat"]["targetSpeed"] = 6.0
    request["observation"]["boat"]["targetCourse"] = 90.0
    return request


class ImmediateStopTests(unittest.TestCase):
    def test_stop_without_waypoints_preserves_all_choices_and_decoding(self):
        request = stop_packet()
        self.assertTrue(valid_request(request))
        obs = request["observation"]
        before = copy.deepcopy(obs)
        payload = build_payload(obs)
        self.assertEqual(payload, build_stop_payload(obs))
        self.assertEqual(obs, before)
        self.assertEqual(payload["state"]["task"]["stage"], "immediate_stop")
        self.assertNotIn("target", payload["state"])
        self.assertNotIn("upcoming_waypoints", payload["state"])
        self.assertEqual(list(payload["questions"]["action"]["criteria"]), list(ACTION_CHOICES))
        for name, (i, j) in ACTION_CHOICES.items():
            result = decode_action({"answers": {"action": {"type": "choice", "choice": name}}}, obs)
            self.assertEqual(result, {"speed": obs["action_space"]["speeds_knots"][i],
                                      "course": obs["action_space"]["courses_deg"][j]})

    def test_stop_validation_and_rest_course(self):
        request = stop_packet()
        request["observation"]["boat"].update(targetSpeed=0, targetCourse=None)
        self.assertTrue(valid_request(request))
        build_payload(request["observation"])
        request["observation"]["task"]["stop_speed_mps"] = -1
        self.assertFalse(valid_request(request))
        request = stop_packet()
        request["type"] = "decision_request"
        self.assertFalse(valid_request(request))
        request = packet()
        request["type"] = "stop_request"
        self.assertFalse(valid_request(request))

    def test_worker_sends_stop_payload_and_uses_model_choice(self):
        sent = []

        class Transport:
            def call(self, payload, *args):
                sent.append(payload)
                # A nonzero answer proves Python does not substitute a local stop action.
                return {"answers": {"action": {"type": "choice", "choice": "speed_1__east"}}}

            def close(self):
                pass

        settings = SimpleNamespace(workers=1, model="test", endpoint="unused", http_transport="persistent")
        scheduler = DecisionScheduler(settings, "unused", transport_factory=Transport)
        try:
            request = stop_packet()
            self.assertTrue(scheduler.submit(request))
            future = next(iter(scheduler.inflight.values()))
            future.result(timeout=5)
            records = scheduler.collect_results()
            self.assertEqual(len(sent), 1)
            self.assertEqual(sent[0]["state"]["task"]["stage"], "immediate_stop")
            self.assertEqual(records[0]["result"]["type"], "decision_result")
            self.assertGreater(records[0]["result"]["action"]["speed"], 0)
        finally:
            scheduler.close()


if __name__ == "__main__":
    unittest.main()
