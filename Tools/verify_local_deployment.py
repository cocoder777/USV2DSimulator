"""Smoke-test packaged scripts at space-containing paths using existing model environments.

Requires both HTTP ports to be unused. Creates junctions in a NEW output directory;
does not reinstall, download, or replace existing deployments. Only stops its own services.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from boat_client import IDENTITY_FIELDS, valid_request  # noqa: E402
from choice_control import decode_action  # noqa: E402
from test_boat_client import packet  # noqa: E402

FLAGS = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0


def port_free(port, udp=False):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM if udp else socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        probe.bind(("127.0.0.1", port))


def script(directory, name, *arguments):
    # A detached service may inherit pipe handles. File output lets the launcher
    # finish without waiting for that service to close an inherited stdout pipe.
    log_path = directory / (name + ".validation.log")
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                                 "-File", str(directory / name), *arguments],
                                stdout=log, stderr=subprocess.STDOUT,
                                creationflags=FLAGS, timeout=330)
    output = log_path.read_text(encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(f"{name} failed: {output}")
    return output


def junction(path, target):
    quote = lambda value: "'" + str(value).replace("'", "''") + "'"
    subprocess.run(["powershell.exe", "-NoProfile", "-Command",
                    f"New-Item -ItemType Junction -Path {quote(path)} -Target {quote(target)} -ErrorAction Stop | Out-Null"],
                   check=True, creationflags=FLAGS, capture_output=True)


def test_backend(backend, original, directory):
    directory.mkdir(parents=True, exist_ok=False)
    for source in (ROOT / "Tools/LocalModels" / backend).iterdir():
        if source.is_file():
            shutil.copy2(source, directory / source.name)
    shutil.copy2(original / "models.json", directory / "models.json")
    (directory / "logs").mkdir()
    (directory / "tmp").mkdir()
    for name in (".venv", "source"):
        junction(directory / name, original / name)
    worker = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        receiver.bind(("127.0.0.1", 25005))
        receiver.settimeout(120)
        try:
            script(directory, f"start-{backend}.ps1")
            health = script(directory, f"check-{backend}.ps1")
            request = packet()
            assert valid_request(request)
            with (directory / "worker.out.log").open("w", encoding="utf-8") as log:
                worker = subprocess.Popen([sys.executable, "-u", str(ROOT / "jev_api/boat_client.py"),
                    "--backend", backend, "--port", "25005", "--listen-port", "25006",
                    "--log", str(directory / "worker.jsonl")], stdout=log, stderr=subprocess.STDOUT,
                    creationflags=FLAGS, env=dict(os.environ, PYTHONUTF8="1"))
                deadline = time.monotonic() + 15
                while "Jev worker listening" not in (directory / "worker.out.log").read_text(encoding="utf-8", errors="replace"):
                    if worker.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("Worker did not bind its UDP port")
                    time.sleep(.1)
                receiver.sendto(json.dumps(request).encode(), ("127.0.0.1", 25006))
                result = json.loads(receiver.recv(8192))
                assert result["type"] == "decision_result", result
                assert all(result[field] == request[field] for field in IDENTITY_FIELDS)
                records = [json.loads(line) for line in (directory / "worker.jsonl").read_text(encoding="utf-8").splitlines()]
                record = records[-1]
                assert result["action"] == decode_action(record["raw_response"])
                assert len(record["payload"]["questions"]["action_0"]["criteria"]) == 56
                assert record["backend"] == backend
                return {"backend": backend, "health": json.loads(health), "action": result["action"],
                        "elapsed_s": record["elapsed_s"], "udp_identity_preserved": True,
                        "all_56_actions_preserved": True, "native_response_matches_udp": True}
        finally:
            if worker and worker.poll() is None:
                worker.terminate()
                worker.wait(timeout=15)
            if (directory / "service-process.json").exists():
                script(directory, f"stop-{backend}.ps1")
            for port in ((8010 if backend == "laya" else 8009), 25006):
                deadline = time.monotonic() + 15
                while True:
                    try:
                        port_free(port, udp=port == 25006)
                        break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise RuntimeError(f"Owned test service/client did not release {port}")
                        time.sleep(.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ai-root", type=Path, default=Path("D:/AI"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for port in (8009, 8010):
        port_free(port)
    for port in (25005, 25006):
        port_free(port, udp=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = []
    try:
        for backend, name in (("laya", "Laya"), ("kev", "Kev-0.8B")):
            result = test_backend(backend, args.ai_root / name, output / "deployment with spaces" / name)
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        (output / "result.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
