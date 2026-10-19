"""Single-action protocol checks, with API calls mocked."""
import copy
import io
import json
import urllib.error
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import boat_client as client


def observation():
    return {
        "coordinates": "Global XY metres; counterclockwise degrees from east.",
        "planning": {"action_duration_s": 0.4, "simulation_paused_while_deciding": True},
        "action_space": {
            "acceleration_fractions": [-1, -2/3, -1/3, 0, .25, .5, .75, 1],
            "turn_fractions": [-1, -.5, -.25, 0, .25, .5, 1]},
        "boat": {"position_xy_m": [0, 0], "heading_deg": 0,
                 "speed_mps": 1, "yaw_rate_deg_s": 8},
        "target": {"position_xy_m": [8, 3], "arrival_radius_m": 1,
                   "requires_stop": True, "stop_speed_mps": .1},
        "dynamics": {"max_acceleration": 1.5, "max_angular_acceleration_deg": 60},
        "motion": "Acceleration commands take effect immediately; motion includes damping.",
    }


def packet():
    return {"type": "decision_request", "version": 5, "tick": 12,
            "session_id": "s", "episode_id": "e", "run_id": "r", "decision_id": "d",
            "observation": observation()}


def response(key="positive_gentle__clockwise_gentle"):
    return {"answers": {"action_0": {"type": "choice", "choice": key}}}


class ChoiceProtocolTests(unittest.TestCase):
    def test_backend_defaults_and_explicit_overrides(self):
        settings = client.parse_settings([])
        self.assertEqual((settings.backend, settings.model, settings.prompt_profile), ("laya", "multilingual", "compact"))
        self.assertEqual(settings.endpoint, "http://127.0.0.1:8010/v1/systemone")
        settings = client.parse_settings(["--backend", "laya"])
        self.assertEqual((settings.backend, settings.model, settings.prompt_profile), ("laya", "multilingual", "compact"))
        for backend in ("kev", "cloud"):
            settings = client.parse_settings(["--backend", backend])
            self.assertEqual(settings.prompt_profile, "original")
            self.assertEqual((settings.endpoint, settings.model), client.BACKENDS[backend][:2])
        settings = client.parse_settings(["--backend", "kev", "--endpoint", "http://localhost:1234/custom", "--model", "custom", "--prompt-profile", "compact"])
        self.assertEqual((settings.endpoint, settings.model, settings.prompt_profile), ("http://localhost:1234/custom", "custom", "compact"))

    def test_compact_keeps_all_action_values_state_and_goal(self):
        obs = observation()
        original_obs = copy.deepcopy(obs)
        compact = client.build_payload(obs, "multilingual", "compact")
        legacy = client.build_payload(obs)
        self.assertEqual(obs, original_obs)
        for field in ("boat", "target", "dynamics", "planning", "motion"):
            self.assertEqual(compact["state"][field], legacy["state"][field])
        criteria = compact["questions"]["action_0"]["criteria"]
        self.assertEqual(set(criteria), set(client.ACTION_CHOICES))
        for key, action in client.ACTION_CHOICES.items():
            a = obs["action_space"]["acceleration_fractions"][action["acceleration_index"]] * obs["dynamics"]["max_acceleration"]
            alpha = obs["action_space"]["turn_fractions"][action["turn_index"]] * obs["dynamics"]["max_angular_acceleration_deg"]
            self.assertEqual(criteria[key], f"a={a:.4g} m/s^2; alpha={alpha:.4g} deg/s^2.")
        self.assertLess(len(json.dumps(compact)), len(json.dumps(legacy)) / 2)
        self.assertIn("stop_speed_mps", compact["questions"]["action_0"]["instructions"])
        obs["target"]["requires_stop"] = False
        checkpoint = client.build_payload(obs, "multilingual", "compact")
        self.assertNotIn("stop_speed_mps", checkpoint["state"]["target"])
        self.assertIn("stopping is not required", checkpoint["questions"]["action_0"]["instructions"])

    def test_laya_budget_and_metadata_logged_without_changing_udp(self):
        settings = client.parse_settings(["--backend", "laya"])
        raw = response()
        raw["usage"] = {"input_tokens": 2000, "output_tokens": 0, "truncated": False}
        log = io.StringIO()
        with patch.object(client, "call_api", return_value=raw) as api, patch("builtins.print"):
            result = client.decide(packet(), settings, None, log)
        payload = api.call_args.args[0]
        self.assertEqual((payload["max_len"], payload["head_max_len"]), (4096, 3072))
        self.assertEqual(result["version"], 5)
        self.assertEqual(result["action"], client.decode_action(raw))
        record = json.loads(log.getvalue())
        self.assertEqual((record["backend"], record["prompt_profile"]), ("laya", "compact"))
        self.assertEqual(record["usage"], raw["usage"])

    def test_local_endpoint_never_uses_cloud_credentials(self):
        with patch.dict("os.environ", {"JEV_API_KEY": "cloud-secret"}):
            for host in ("127.0.0.1", "localhost", "[::1]"):
                self.assertIsNone(client.api_key_for_endpoint(f"http://{host}:8009/v1/systemone"))
            self.assertEqual(client.api_key_for_endpoint("https://example.invalid/v1/systemone"), "cloud-secret")
        with patch.dict("os.environ", {}, clear=True):
            with self.assertRaisesRegex(ValueError, "JEV_API_KEY"):
                client.api_key_for_endpoint("https://example.invalid/v1/systemone")
        for endpoint in ("http://example.invalid", "file:///tmp/model", "http://user:password@localhost:8009"):
            with self.assertRaises(ValueError):
                client.api_key_for_endpoint(endpoint)
        with patch.object(client.urllib.request, "urlopen") as urlopen:
            urlopen.return_value.__enter__.return_value.read.return_value = b'{}'
            client.call_api({}, "cloud-secret", "http://127.0.0.1:8009/v1/systemone")
            self.assertIsNone(urlopen.call_args.args[0].get_header("Authorization"))

    def test_unavailable_service_has_clear_error(self):
        with patch.object(client.urllib.request, "urlopen", side_effect=urllib.error.URLError("connection refused")):
            with self.assertRaisesRegex(RuntimeError, "Decision service unavailable"):
                client.call_api({}, None, "http://127.0.0.1:8009/v1/systemone")

    def test_one_question_with_all_joint_options_and_current_duration(self):
        obs = observation()
        for duration in (.4, 1):
            obs["planning"]["action_duration_s"] = duration
            payload = client.build_payload(obs)
            self.assertEqual(set(payload["questions"]), {"action_0"})
            question = payload["questions"]["action_0"]
            self.assertEqual(question["type"], "choice")
            self.assertEqual(set(question["criteria"]), set(client.ACTION_CHOICES))
            self.assertEqual(len(question["criteria"]), 56)
            self.assertIn(f"next {duration:g} seconds", question["instructions"])
            self.assertNotIn("batch", question["instructions"])
            self.assertNotIn("sequence", json.dumps(payload))
            self.assertNotIn("action_count", json.dumps(payload))
        options = question["criteria"]
        self.assertIn("longitudinal -1.5 m/s^2", options["negative_full__clockwise_full"])
        self.assertIn("angular 60 deg/s^2", options["positive_full__counterclockwise_full"])

    def test_model_state_is_detached_and_contains_only_current_body_target(self):
        obs = observation()
        original = copy.deepcopy(obs)
        obs["transport_only"] = {"must_not_reach_model": True}
        payload = client.build_payload(obs)
        self.assertEqual({k: v for k, v in obs.items() if k != "transport_only"}, original)
        self.assertEqual(set(payload["state"]), set(client.MODEL_STATE_FIELDS))
        self.assertNotIn("must_not_reach_model", json.dumps(payload))
        self.assertEqual(set(payload["state"]["boat"]), {"speed_mps", "yaw_rate_deg_s"})
        self.assertNotIn("position_xy_m", payload["state"]["target"])
        payload["state"]["boat"]["speed_mps"] = 99
        self.assertEqual(obs["boat"]["speed_mps"], 1)

    def test_body_transform_signs_and_rotation(self):
        for heading, forward, left in [(0, 10, 0), (90, 0, -10), (180, -10, 0),
                                       (270, 0, 10), (45, 2**.5*5, -2**.5*5)]:
            obs = observation()
            obs["boat"]["heading_deg"] = heading
            obs["target"]["position_xy_m"] = [10, 0]
            target = client.build_payload(obs)["state"]["target"]
            self.assertAlmostEqual(target["forward_m"], forward, places=5)
            self.assertAlmostEqual(target["left_m"], left, places=5)

    def test_checkpoint_and_stop_target_contract(self):
        for stop in (False, True):
            obs = observation()
            obs["target"]["requires_stop"] = stop
            payload = client.build_payload(obs)
            self.assertNotIn("requires_stop", json.dumps(payload))
            self.assertEqual(obs["target"]["requires_stop"], stop)
            self.assertEqual("stop_speed_mps" in payload["state"]["target"], stop)
            text = payload["questions"]["action_0"]["instructions"]
            self.assertEqual("FINAL PARKING REGION" in text, stop)
            self.assertEqual("is a CHECKPOINT" in text, not stop)
            self.assertEqual("target.stop_speed_mps" in text, stop)

    def test_all_joint_choices_map_to_one_existing_index_pair(self):
        actual = set()
        for key, expected in client.ACTION_CHOICES.items():
            decoded = client.decode_action(response(key))
            self.assertEqual(decoded, expected)
            actual.add((decoded["acceleration_index"], decoded["turn_index"]))
        self.assertEqual(actual, {(a, t) for a in range(8) for t in range(7)})
        decoded = client.decode_action(response())
        decoded["acceleration_index"] = 99
        self.assertEqual(client.decode_action(response())["acceleration_index"], 4)

    def test_malformed_choices_and_extra_answers_fail(self):
        for key in (None, True, 3, 3.0, {}, [], "", "a7_t0", "positive_full__unknown"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                client.decode_action(response(key))
        extra = response()
        extra["answers"]["action_1"] = extra["answers"]["action_0"]
        for raw in (None, [], {}, {"answers": []}, {"answers": {}}, {"answers": {"wrong": {}}},
                    {"answers": {"action_0": {"type": "score", "score": 3}}},
                    {"answers": {"action_0": None}}, extra):
            with self.assertRaises(ValueError):
                client.decode_action(raw)

    def test_valid_request_rejects_old_protocol_and_action_count(self):
        self.assertTrue(client.valid_request(packet()))
        old = packet()
        old["version"] = 4
        self.assertFalse(client.valid_request(old))
        for count in (1, 3, 10):
            old = packet()
            old["observation"]["planning"]["action_count"] = count
            self.assertFalse(client.valid_request(old))
        old = packet()
        old["observation"]["actions"] = old["observation"].pop("action_space")
        self.assertFalse(client.valid_request(old))

    def test_request_duration_and_identity_validation(self):
        for duration in (0, -1, True, "0.4", None, float("nan"), float("inf")):
            invalid = packet()
            invalid["observation"]["planning"]["action_duration_s"] = duration
            self.assertFalse(client.valid_request(invalid))
        for key, value in (("tick", True), ("tick", -1), ("decision_id", ""), ("version", 5.0)):
            invalid = packet()
            invalid[key] = value
            self.assertFalse(client.valid_request(invalid))

    def test_decide_preserves_identity_logs_payload_and_returns_single_action(self):
        request = packet()
        log = io.StringIO()
        settings = SimpleNamespace(model="jev-latest", endpoint="https://example.invalid")
        with patch.object(client, "call_api", return_value=response()) as api, patch("builtins.print"):
            result = client.decide(request, settings, "test-key", log)
        self.assertEqual(result["type"], "decision_result")
        self.assertEqual(result["version"], 5)
        self.assertEqual(result["action_duration_s"], .4)
        self.assertEqual(result["action"], {"acceleration_index": 4, "turn_index": 2})
        self.assertNotIn("actions", result)
        for field in client.IDENTITY_FIELDS:
            self.assertEqual(result[field], request[field])
        record = json.loads(log.getvalue())
        self.assertEqual(record["payload"], api.call_args.args[0])
        self.assertEqual(record["result"], result)
        self.assertEqual(record["raw_response"], response())
        self.assertIn("position_xy_m", record["request"]["observation"]["boat"])
        self.assertNotIn("position_xy_m", record["payload"]["state"]["boat"])
        self.assertNotIn("test-key", log.getvalue())

    def test_api_failure_returns_matching_error_without_action(self):
        log = io.StringIO()
        settings = SimpleNamespace(model="jev-latest", endpoint="https://example.invalid")
        with patch.object(client, "call_api", side_effect=RuntimeError("unavailable")), patch("builtins.print"):
            result = client.decide(packet(), settings, "test-key", log)
        self.assertEqual(result["type"], "decision_error")
        self.assertEqual(result["version"], 5)
        self.assertNotIn("action", result)
        self.assertIn("unavailable", result["error"])


if __name__ == "__main__":
    unittest.main()
