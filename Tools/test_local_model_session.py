import argparse
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import local_model_session as session


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent)
        self.directory = Path(self.temp.name)
        args = argparse.Namespace(backend="laya", session_dir=self.directory / "session", ai_root=self.directory,
                                  project=self.directory, parent_pid=123, unity_port=15005, worker_port=15006)
        self.owner_patch = patch.object(session, "ProcessHandle")
        owner = self.owner_patch.start()
        owner.return_value.alive.return_value = True
        self.manager = session.Session(args)
        self.lock_patch = patch.object(self.manager, "acquire_deployments")
        self.lock_patch.start()

    def tearDown(self):
        self.owner_patch.stop()
        self.lock_patch.stop()
        self.temp.cleanup()

    def test_rejects_cpu_or_wrong_revision(self):
        manifest = {"revision": "pinned"}
        for health in ({"revision": "other", "device": "cuda", "budget_guard": True},
                       {"revision": "pinned", "device": "cpu", "budget_guard": True},
                       {"revision": "pinned", "device": "cuda", "budget_guard": False}):
            with self.assertRaises(RuntimeError):
                session.validate_health("laya", manifest, health)
        session.validate_health("laya", manifest, {"revision": "pinned", "device": "cuda:0", "budget_guard": True})

    def test_cloud_never_starts_local_services(self):
        self.manager.args.backend = "cloud"
        with patch.dict("os.environ", {"JEV_API_KEY": "test-only"}), patch.object(self.manager, "script") as script, patch.object(self.manager, "health") as health:
            self.manager.ensure_service()
        script.assert_not_called()
        health.assert_not_called()
        self.assertFalse(self.manager.service_started)

    def test_cloud_missing_key_preserves_existing_client(self):
        self.manager.args.backend = "cloud"
        with patch.dict("os.environ", {}, clear=True), patch.object(self.manager, "take_over_project_worker") as takeover:
            self.manager.run()
        takeover.assert_not_called()
        self.assertEqual(self.manager.status["phase"], "error")
        self.assertIn("JEV_API_KEY", self.manager.status["message"])

    def test_cloud_command_pins_original_repository_request(self):
        self.manager.args.backend = "cloud"
        command = self.manager.worker_command(Path("python.exe"), Path("boat_client.py"))
        for flag, value in (("--backend", "cloud"), ("--endpoint", "https://api.typesafe.ai/v1/systemone"),
                            ("--model", "jev-latest"), ("--prompt-profile", "original")):
            self.assertEqual(command[command.index(flag) + 1], value)
        self.assertNotIn("JEV_API_KEY", " ".join(command))

    def test_kev_requires_original_run_and_cuda(self):
        manifest = {"kev_path": "weights", "base_id": "base"}
        with self.assertRaises(RuntimeError):
            session.validate_health("kev", manifest, {"models": [{"run": "other", "base": "base", "device": "cuda"}]})
        session.validate_health("kev", manifest, {"models": [{"run": "weights", "base": "base", "device": "cuda"}]})

    def test_port_conflict_does_not_touch_any_service(self):
        with patch.object(session, "udp_available", return_value=False), patch.object(session, "inspect_udp_owner", return_value=[]), patch.object(self.manager, "ensure_service") as start:
            self.manager.run()
        start.assert_not_called()
        status = json.loads((self.manager.directory / "status.json").read_text(encoding="utf-8"))
        self.assertEqual(status["phase"], "error")
        self.assertIn("15006", status["message"])

    def test_stop_and_owner_exit_cancel_session(self):
        self.assertFalse(self.manager.cancelled())
        (self.manager.directory / "stop").touch()
        with self.assertRaises(session.Cancelled):
            self.manager.check_cancelled()
        (self.manager.directory / "stop").unlink()
        self.manager.owner.alive.return_value = False
        self.assertTrue(self.manager.cancelled())

    def test_reused_service_is_not_stopped_on_cleanup(self):
        with patch.object(self.manager, "ensure_service", side_effect=RuntimeError("probe failed")), patch.object(self.manager, "script") as script, patch.object(session, "udp_available", return_value=True):
            self.manager.run()
        script.assert_not_called()

    def test_started_service_is_stopped_if_worker_cannot_start(self):
        def started():
            self.manager.service_started = True
        with patch.object(self.manager, "ensure_service", side_effect=started), patch.object(self.manager, "owns_current_service", return_value=True), patch.object(self.manager, "script") as script, patch.object(session, "udp_available", return_value=True):
            self.manager.run()  # Missing Python/client yields a controlled error.
        script.assert_called_once_with(self.manager.service_directory, "stop", "laya", timeout=30, cancellable=False)

    def test_reports_latency_without_changing_raw_worker_output(self):
        text = 'Jev worker listening on test\n' + json.dumps({"event": "decision_result", "elapsed_s": .123,
                "action": {"acceleration_index": 2, "turn_index": 0}}) + "\n"
        path = self.manager.directory / "worker.out.log"
        path.write_text(text, encoding="utf-8")
        self.manager.read_worker_updates()
        self.assertEqual(self.manager.status["last_decision_s"], .123)
        self.assertEqual(self.manager.status["last_action"], "(2, 0)")
        self.manager.read_worker_updates()
        self.assertEqual(self.manager.status["decision_count"], 1)
        self.assertEqual(path.read_text(encoding="utf-8"), text)

    def test_project_client_requires_absolute_path_and_same_ports(self):
        project = Path("D:/Unity/project/USV2DSimulator")
        valid = ["python.exe", "-u", "D:/Unity/project/USV2DSimulator/jev_api/boat_client.py", "--backend", "laya"]
        self.assertTrue(session.project_worker_matches(valid, project, 5005, 5006))
        self.assertFalse(session.project_worker_matches(["python.exe", "jev_api/boat_client.py"], project, 5005, 5006))
        self.assertFalse(session.project_worker_matches(valid, project, 15005, 15006))
        self.assertFalse(session.project_worker_matches(valid + ["--port", "1", "--port", "5005"], project, 5005, 5006))

    def test_unrelated_or_reused_pid_is_never_killed(self):
        record = {"pid": 321, "executable": "other.exe", "command": "unrelated", "start_ticks": "123"}
        with patch.object(session, "inspect_udp_owner", return_value=[record]), patch.object(session, "command_arguments", return_value=[]), patch.object(session.subprocess, "run") as kill:
            with self.assertRaises(RuntimeError):
                self.manager.take_over_project_worker()
        kill.assert_not_called()
        record["executable"] = "python.exe"
        with patch.object(session, "inspect_udp_owner", return_value=[record]), patch.object(session, "command_arguments", return_value=[]), patch.object(session, "project_worker_matches", return_value=True), patch.object(session.subprocess, "run") as kill:
            self.manager.owner.start_ticks.return_value = 456
            with self.assertRaisesRegex(RuntimeError, "identity changed"):
                self.manager.take_over_project_worker()
        kill.assert_not_called()

    def test_cleanup_does_not_stop_a_replaced_service(self):
        self.manager.service_directory.mkdir()
        self.manager.service_identity = {"pid": 1, "start_ticks": "100"}
        state = self.manager.service_directory / "service-process.json"
        state.write_text(json.dumps({"pid": 2, "start_ticks": "200"}), encoding="utf-8")
        self.assertFalse(self.manager.owns_current_service())
        state.write_text(json.dumps(self.manager.service_identity), encoding="utf-8")
        self.assertTrue(self.manager.owns_current_service())

    def test_only_confirmed_project_worker_is_taken_over(self):
        record = {"pid": 321, "executable": "python.exe", "command": "known", "start_ticks": "123"}
        self.manager.owner.start_ticks.return_value = 123
        with patch.object(session, "inspect_udp_owner", return_value=[record]), patch.object(session, "command_arguments", return_value=[]), patch.object(session, "project_worker_matches", return_value=True), patch.object(session, "udp_available", return_value=True), patch.object(session.subprocess, "run") as kill:
            self.manager.take_over_project_worker()
        self.assertEqual(kill.call_args.args[0], ["taskkill", "/PID", "321", "/T", "/F"])

    def test_status_publish_retries_windows_file_sharing(self):
        original = session.os.replace
        calls = []
        def replace(source, target):
            calls.append(source)
            if len(calls) == 1:
                raise PermissionError("sharing violation")
            return original(source, target)
        with patch.object(session.os, "replace", side_effect=replace):
            self.manager.publish("ready", "test")
        self.assertEqual(len(calls), 2)
        status = json.loads((self.manager.directory / "status.json").read_text(encoding="utf-8"))
        self.assertEqual(status["phase"], "ready")

    def test_terminate_only_owned_live_process_tree(self):
        process = Mock(pid=321)
        process.poll.return_value = None
        with patch.object(session.subprocess, "run") as kill:
            session.terminate_owned(process)
        if sys.platform == "win32":
            self.assertEqual(kill.call_args.args[0], ["taskkill", "/PID", "321", "/T", "/F"])


if __name__ == "__main__":
    unittest.main()
