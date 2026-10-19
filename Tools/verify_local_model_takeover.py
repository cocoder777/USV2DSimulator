"""Real Windows ownership/port checks for the model-button session manager."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time

import local_model_session as manager


def wait_for(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.1)
    raise RuntimeError("Validation deadline exceeded")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    python = Path("D:/AI/Laya/.venv/Scripts/python.exe")
    parent = None
    helper = None
    session_dir = output / "takeover"
    try:
        with (output / "old-client.log").open("w", encoding="utf-8") as log:
            parent = subprocess.Popen([str(python), "-u", str(root / "jev_api/boat_client.py"),
                "--backend", "laya", "--port", "16005", "--listen-port", "16006"],
                stdout=log, stderr=subprocess.STDOUT, creationflags=manager.FLAGS)
            wait_for(lambda: "Jev worker listening" in (output / "old-client.log").read_text(encoding="utf-8", errors="replace"), 15)
            helper = subprocess.Popen([str(python), str(root / "Tools/local_model_session.py"),
                "--backend", "laya", "--project", str(root), "--session-dir", str(session_dir),
                "--parent-pid", str(os.getpid()), "--unity-port", "16005", "--worker-port", "16006"], creationflags=manager.FLAGS)
            def ready():
                path = session_dir / "status.json"
                if not path.exists():
                    return False
                status = json.loads(path.read_text(encoding="utf-8"))
                if status["phase"] == "error":
                    raise RuntimeError(status["message"])
                return status["phase"] == "ready"
            wait_for(ready, 180)
            wait_for(lambda: parent.poll() is not None, 10)
            assert not manager.udp_available(16006)
            (session_dir / "stop").touch()
            helper.wait(timeout=45)
            assert manager.udp_available(16006)
        # Keep an unrelated socket owned by this test, verify it is never terminated.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as unrelated:
            unrelated.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            unrelated.bind(("127.0.0.1", 16006))
            rejected = output / "unrelated"
            check = subprocess.run([str(python), str(root / "Tools/local_model_session.py"),
                "--backend", "laya", "--project", str(root), "--session-dir", str(rejected),
                "--parent-pid", str(os.getpid()), "--unity-port", "16005", "--worker-port", "16006"],
                creationflags=manager.FLAGS, timeout=30)
            status = json.loads((rejected / "status.json").read_text(encoding="utf-8"))
            assert status["phase"] == "error"
            assert not manager.udp_available(16006)
            assert unrelated.getsockname() == ("127.0.0.1", 16006)
        result = {"project_client_taken_over": True, "unrelated_socket_preserved": True,
                  "worker_port_released": manager.udp_available(16006)}
        (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result))
    finally:
        if session_dir.exists():
            (session_dir / "stop").touch()
        if helper and helper.poll() is None:
            helper.wait(timeout=45)
        manager.terminate_owned(parent)


if __name__ == "__main__":
    main()
