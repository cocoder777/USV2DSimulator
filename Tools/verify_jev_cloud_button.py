"""Real Unity/UDP worker test with a local HTTP double; never calls paid cloud API."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--test-project", type=Path, required=True)
    parser.add_argument("--unity", type=Path, default=Path("D:/Unity/6000.6.5f1/Editor/Unity.exe"))
    parser.add_argument("--timeout", type=float, default=180, help="Allow extra time for a cold Unity asset import")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    records = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            record = {"model": data.get("model"), "choices": len(data["questions"]["action_0"]["criteria"]),
                "original_profile": "max_len" not in data and "head_max_len" not in data,
                "test_credential_only": self.headers.get("Authorization") == "Bearer validation-only-no-cloud-access"}
            records.append(record)
            if (output / "fail-http").exists():
                self.send_error(503)
                return
            body = json.dumps({"answers": {"action_0": {"type": "choice", "choice": "positive_full__no_angular_input"}}}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    hooks = output / "test-hooks"
    hooks.mkdir()
    # Only the validation process inherits this hook; production remains canonical HTTPS.
    (hooks / "sitecustomize.py").write_text(
        "import urllib.request\n"
        "_original = urllib.request.urlopen\n"
        "def _validation_open(request, *args, **kwargs):\n"
        "    if getattr(request, 'full_url', '') == 'https://api.typesafe.ai/v1/systemone':\n"
        f"        request.full_url = 'http://127.0.0.1:{server.server_port}/v1/systemone'\n"
        "    return _original(request, *args, **kwargs)\n"
        "urllib.request.urlopen = _validation_open\n", encoding="utf-8")
    env = dict(os.environ, PYTHONPATH=str(hooks), USV_JEV_VALIDATION_OUTPUT=str(output / "checks.jsonl"),
               USV_JEV_PROJECT=str(args.test_project.resolve()))
    env.pop("JEV_API_KEY", None)
    try:
        process = subprocess.Popen([str(args.unity), "-batchmode", "-nographics", "-projectPath", str(args.test_project),
            "-executeMethod", "JevCloudValidation.Run", "--jev-cloud-validation", "-logFile", str(output / "unity.log")],
            env=env, creationflags=subprocess.CREATE_NO_WINDOW)
        code = process.wait(timeout=args.timeout)
        checks = [json.loads(line) for line in (output / "checks.jsonl").read_text(encoding="utf-8").splitlines()]
        valid_requests = bool(records) and all(r["model"] == "jev-latest" and r["choices"] == 56
            and r["original_profile"] and r["test_credential_only"] for r in records)
        worker_logs = sorted((args.test_project / "Logs/model-switch").glob("*/worker.jsonl"), key=lambda p: p.stat().st_mtime)
        latest = worker_logs[-1]
        decisions = [json.loads(line) for line in latest.read_text(encoding="utf-8").splitlines()]
        baseline_source = subprocess.check_output(["git", "show", "42088e5:jev_api/choice_control.py"], cwd=ROOT, encoding="utf-8")
        baseline = {}
        exec(compile(baseline_source, "repository_choice_control.py", "exec"), baseline)
        exact = bool(decisions) and all(d["payload"] == baseline["build_payload"](d["request"]["observation"], "jev-latest") for d in decisions)
        result = {"unity_exit_code": code, "passed": sum(c["passed"] for c in checks), "total": len(checks),
            "http_requests": records, "exact_repository_payloads": exact,
            "real_cloud_called": False, "worker_log": str(latest)}
        (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False))
        if code or not all(c["passed"] for c in checks) or not valid_requests or not exact:
            raise SystemExit(2)
    finally:
        if 'process' in locals() and process.poll() is None:
            process.terminate()
        server.shutdown()


if __name__ == "__main__":
    main()
