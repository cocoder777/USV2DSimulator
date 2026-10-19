"""Compare cloud requests to the verified GitHub baseline without reading any key."""
import copy
import io
import json
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
import boat_client as client
from test_boat_client import packet, response

BASELINE = "42088e5d891a701ff838bef45be5445977ed769f"


class RepositoryJevTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = subprocess.check_output(["git", "show", BASELINE + ":jev_api/choice_control.py"],
                                         cwd=ROOT, encoding="utf-8")
        cls.original = types.ModuleType("repository_choice_control")
        exec(compile(source, "repository_choice_control.py", "exec"), cls.original.__dict__)

    def test_cloud_payload_and_decoding_match_github_for_24_states(self):
        settings = client.parse_settings(["--backend", "cloud"])
        self.assertEqual(settings.model, "jev-latest")
        self.assertEqual(settings.endpoint, "https://api.typesafe.ai/v1/systemone")
        self.assertEqual(settings.prompt_profile, "original")
        for speed in (-1, 0, 1.5):
            for heading in (0, 90, 180, 270):
                for stop in (False, True):
                    request = copy.deepcopy(packet())
                    obs = request["observation"]
                    obs["boat"].update(speed_mps=speed, heading_deg=heading)
                    obs["target"]["requires_stop"] = stop
                    expected = self.original.build_payload(obs, "jev-latest")
                    log = io.StringIO()
                    raw = response()
                    with patch.object(client, "call_api", return_value=raw) as api, patch("builtins.print"):
                        result = client.decide(request, settings, "test-only", log)
                    self.assertEqual(api.call_args.args, (expected, "test-only", settings.endpoint))
                    self.assertNotIn("max_len", expected)
                    self.assertNotIn("head_max_len", expected)
                    self.assertEqual(len(expected["questions"]["action_0"]["criteria"]), 56)
                    self.assertEqual(result["action"], self.original.decode_action(raw))
                    self.assertEqual(json.loads(log.getvalue())["backend"], "cloud")

    def test_every_legacy_action_mapping_is_preserved(self):
        self.assertEqual(client.ACTION_CHOICES, self.original.ACTION_CHOICES)
        for label in client.ACTION_CHOICES:
            self.assertEqual(client.decode_action(response(label)), self.original.decode_action(response(label)))


if __name__ == "__main__":
    unittest.main()
