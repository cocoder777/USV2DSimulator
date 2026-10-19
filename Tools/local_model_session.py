"""Own one Unity button-started cloud/local worker; reuse pinned Python environments.

Status/stop files are private to a Unity session. No changes to model inputs or UDP.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import ntpath
import socket
import subprocess
import time
import urllib.request

BACKENDS = {"laya": ("Laya", 8010, "deployment"), "kev": ("Kev-0.8B", 8009, "v1/models")}
CLOUD = ("https://api.typesafe.ai/v1/systemone", "jev-latest", "original")
FLAGS = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0


class Cancelled(Exception):
    pass


def terminate_owned(process):
    if process is None or process.poll() is not None:
        return
    if os.name == "nt":
        # A venv launcher may own an actual interpreter child. Only this Popen tree.
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       capture_output=True, creationflags=FLAGS, timeout=15)
    else:
        process.terminate()
    process.wait(timeout=15)


def udp_available(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def tcp_listening(port):
    with socket.socket() as probe:
        probe.settimeout(.5)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def validate_health(backend, manifest, health):
    if backend == "laya":
        valid = (health.get("revision") == manifest["revision"] and
                 health.get("device", "").startswith("cuda") and health.get("budget_guard") is True)
    else:
        models = health.get("models") or []
        card = models[0] if models else {}
        valid = (card.get("run") == manifest["kev_path"] and card.get("base") == manifest["base_id"]
                 and card.get("device", "").startswith("cuda"))
    if not valid:
        raise RuntimeError(f"Unexpected {backend} model identity/device. Inspect the deployment service.")


class ProcessHandle:
    """Keep a Windows handle open so a reused PID cannot impersonate the owner."""
    def __init__(self, pid):
        self.kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        self.kernel.OpenProcess.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        self.kernel.OpenProcess.restype = ctypes.c_void_p
        self.kernel.GetExitCodeProcess.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
        self.kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        self.handle = self.kernel.OpenProcess(0x1000, False, pid)

    def alive(self):
        code = ctypes.c_ulong()
        return bool(self.handle and self.kernel.GetExitCodeProcess(self.handle, ctypes.byref(code)) and code.value == 259)

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle)
            self.handle = None

    def start_ticks(self):
        self.kernel.GetProcessTimes.argtypes = [ctypes.c_void_p] + [ctypes.POINTER(ctypes.c_ulonglong)] * 4
        values = [ctypes.c_ulonglong() for _ in range(4)]
        if not self.handle or not self.kernel.GetProcessTimes(self.handle, *(ctypes.byref(value) for value in values)):
            return None
        return values[0].value + 504911232000000000  # FILETIME 1601 -> .NET 0001.


def command_arguments(command):
    shell = ctypes.WinDLL("shell32", use_last_error=True)
    shell.CommandLineToArgvW.argtypes = [ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_int)]
    shell.CommandLineToArgvW.restype = ctypes.POINTER(ctypes.c_wchar_p)
    count = ctypes.c_int()
    array = shell.CommandLineToArgvW(command, ctypes.byref(count))
    if not array:
        return []
    try:
        return [array[index] for index in range(count.value)]
    finally:
        kernel = ctypes.WinDLL("kernel32")
        kernel.LocalFree.argtypes = [ctypes.c_void_p]
        kernel.LocalFree(array)


def project_worker_matches(arguments, project, unity_port, worker_port):
    # Require an absolute, exact script path. Relative command lines are ambiguous.
    script = ntpath.normcase(ntpath.abspath(str(project / "jev_api/boat_client.py")))
    matches = [index for index, value in enumerate(arguments)
               if ntpath.isabs(value) and ntpath.normcase(ntpath.normpath(value)) == script]
    if len(matches) != 1:
        return False
    options = arguments[matches[0] + 1:]
    def option(name, default):
        occurrences = [index for index, value in enumerate(options) if value == name]
        if len(occurrences) > 1:
            return None
        if not occurrences:
            if any(value.startswith(name + "=") for value in options):
                return None  # Conservative: do not guess alternate/duplicate syntax.
            return default
        index = occurrences[0]
        return options[index + 1] if index + 1 < len(options) else None
    return (option("--host", "127.0.0.1") == "127.0.0.1" and
            option("--listen-host", "127.0.0.1") == "127.0.0.1" and
            option("--port", "5005") == str(unity_port) and
            option("--listen-port", "5006") == str(worker_port))


def inspect_udp_owner(port):
    # Read-only inspection; no command line is printed or written into a status file.
    script = ("[Console]::OutputEncoding=[Text.UTF8Encoding]::new($false); "
              f"$items=@(Get-NetUDPEndpoint -LocalPort {int(port)} -ErrorAction SilentlyContinue | "
              "Where-Object {$_.LocalAddress -eq '127.0.0.1'} | Select-Object -ExpandProperty OwningProcess -Unique); "
              "$result=@(foreach($processId in $items) { $details=Get-CimInstance Win32_Process -Filter \"ProcessId=$processId\"; "
              "$process=Get-Process -Id $processId -ErrorAction SilentlyContinue; "
              "if($details -and $process) { [pscustomobject]@{pid=$processId; executable=$details.ExecutablePath; "
              "command=$details.CommandLine; start_ticks=$process.StartTime.ToUniversalTime().Ticks.ToString()} } }); "
              "ConvertTo-Json -InputObject $result -Compress")
    result = subprocess.run(["powershell.exe", "-NoProfile", "-Command", script],
                            capture_output=True, text=True, encoding="utf-8", errors="replace",
                            creationflags=FLAGS, timeout=15)
    if result.returncode:
        return []
    return json.loads(result.stdout.strip() or "[]")


class Session:
    def __init__(self, args):
        self.args = args
        self.directory = args.session_dir.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.status = {"backend": args.backend, "phase": "starting", "message": "Preparing model...",
                       "helper_pid": os.getpid(),
                       "worker_log": str(self.directory / "worker.jsonl"), "decision_count": 0,
                       "last_decision_s": -1.0, "last_action": "--"}
        self.owner = ProcessHandle(args.parent_pid)
        self.worker = None
        self.service_started = False
        self.service_identity = None
        self.deployment_lock = None
        self.service_directory = args.ai_root / BACKENDS.get(args.backend, ("Laya",))[0]
        self.stdout_position = 0

    def publish(self, phase, message, **extra):
        self.status.update(phase=phase, message=message, **extra)
        self.status["updated_at"] = time.time()
        temporary = self.directory / "status.tmp"
        temporary.write_text(json.dumps(self.status, ensure_ascii=False), encoding="utf-8")
        for attempt in range(20):
            try:
                os.replace(temporary, self.directory / "status.json")
                break
            except PermissionError:
                if attempt == 19:
                    raise
                time.sleep(.01)  # Brief Windows reader/antivirus sharing window.

    def cancelled(self):
        return (self.directory / "stop").exists() or not self.owner.alive()

    def check_cancelled(self):
        if self.cancelled():
            raise Cancelled()

    def acquire_deployments(self):
        # One GPU/model switch owner per checkout, including multiple Editor windows.
        import msvcrt
        path = self.args.project / "Logs/model-switch.lock"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.deployment_lock = path.open("a+b")
        self.deployment_lock.seek(0, os.SEEK_END)
        if self.deployment_lock.tell() == 0:
            self.deployment_lock.write(b"0")
            self.deployment_lock.flush()
        deadline = time.monotonic() + 330
        while True:
            self.check_cancelled()
            try:
                self.deployment_lock.seek(0)
                msvcrt.locking(self.deployment_lock.fileno(), msvcrt.LK_NBLCK, 1)
                return
            except OSError:
                self.publish("starting", "Waiting for another model-button session to close...")
                if time.monotonic() >= deadline:
                    raise RuntimeError("Another model-button session owns the GPU services. Stop its Play session first.")
                time.sleep(.2)

    def owns_current_service(self):
        if not self.service_identity:
            return False
        path = self.service_directory / "service-process.json"
        if not path.exists():
            return False
        record = json.loads(path.read_text(encoding="utf-8-sig"))
        return all(str(record.get(key)) == str(self.service_identity.get(key)) for key in ("pid", "start_ticks"))

    def script(self, directory, verb, backend, timeout=330, cancellable=True):
        script = directory / f"{verb}-{backend}.ps1"
        if not script.is_file():
            raise RuntimeError(f"Missing deployment script: {script}")
        path = self.directory / f"{verb}-{backend}.log"
        with path.open("a", encoding="utf-8") as log:
            process = subprocess.Popen(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                                        "-File", str(script)], stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=FLAGS)
            try:
                deadline = time.monotonic() + timeout
                while process.poll() is None:
                    if cancellable:
                        self.check_cancelled()
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f"{verb} {backend} timed out; inspect {path}")
                    time.sleep(.1)
                if process.returncode:
                    raise RuntimeError(f"{verb} {backend} failed; inspect {path}")
            finally:
                # Do not /T kill a service that deliberately runs beyond the startup script.
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)

    def health(self):
        _, port, resource = BACKENDS[self.args.backend]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/{resource}", timeout=3) as response:
            health = json.load(response)
        manifest = json.loads((self.service_directory / "models.json").read_text(encoding="utf-8-sig"))
        validate_health(self.args.backend, manifest, health)

    def ensure_service(self):
        backend = self.args.backend
        if backend == "cloud":
            # No CUDA health check, weight load, or fallback for remote Jev.
            if not os.environ.get("JEV_API_KEY", "").strip():
                raise RuntimeError("Cloud Jev requires JEV_API_KEY; no local-model fallback.")
            return
        other = "kev" if backend == "laya" else "laya"
        other_directory = self.args.ai_root / BACKENDS[other][0]
        self.publish("starting", "Releasing the other model's GPU memory...")
        # Deployment stop scripts validate PID, start time and executable identity.
        # Never kill arbitrary listeners or processes with a matching name.
        if (other_directory / "service-process.json").exists():
            self.script(other_directory, "stop", other, timeout=30)
        if tcp_listening(BACKENDS[other][1]):
            raise RuntimeError(f"Port {BACKENDS[other][1]} belongs to an unmanaged service. Stop it manually first.")
        self.check_cancelled()
        if tcp_listening(BACKENDS[backend][1]):
            self.health()  # A wrong model/CPU service is an error, never a silent fallback.
            return
        self.publish("starting", f"Loading {backend}; this can take a minute...")
        state_file = self.service_directory / "service-process.json"
        existing = None
        if state_file.exists():
            record = json.loads(state_file.read_text(encoding="utf-8-sig"))
            existing = ProcessHandle(int(record["pid"]))
        try:
            if existing and existing.alive():
                deadline = time.monotonic() + 300
                while not tcp_listening(BACKENDS[backend][1]):
                    self.check_cancelled()
                    if not existing.alive() or time.monotonic() >= deadline:
                        raise RuntimeError(f"Existing {backend} service did not become ready; inspect deployment logs.")
                    time.sleep(.2)
            else:
                # Complete an in-flight load before handling cancellation so the
                # newly-created service's PID is known and can be cleaned safely.
                self.script(self.service_directory, "start", backend, cancellable=False)
                self.service_identity = json.loads(state_file.read_text(encoding="utf-8-sig"))
                self.service_started = True
            self.health()
        finally:
            if existing:
                existing.close()

    def read_worker_updates(self):
        path = self.directory / "worker.out.log"
        with path.open(encoding="utf-8", errors="replace") as log:
            log.seek(self.stdout_position)
            lines = log.readlines()
            self.stdout_position = log.tell()
        for line in lines:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if record.get("event") not in ("decision_result", "decision_error"):
                continue
            action = record.get("action") or {}
            self.publish("ready", record.get("error") or f"{self.args.backend} running",
                decision_count=self.status["decision_count"] + 1,
                last_decision_s=record.get("elapsed_s", -1),
                last_action=f'({action.get("acceleration_index", "--")}, {action.get("turn_index", "--")})')

    def run(self):
        try:
            self.check_cancelled()
            if self.args.backend == "cloud":
                self.ensure_service()  # Validate credentials before replacing any running client.
            self.acquire_deployments()
            if not udp_available(self.args.worker_port):
                self.take_over_project_worker()
            self.ensure_service()
            self.check_cancelled()
            python = self.service_directory / ".venv/Scripts/python.exe"
            if self.args.backend == "cloud" and not python.is_file():
                python = self.args.ai_root / "Kev-0.8B/.venv/Scripts/python.exe"
            client = self.args.project / "jev_api/boat_client.py"
            if not python.is_file() or not client.is_file():
                raise RuntimeError("Missing deployment Python or project boat_client.py")
            self.publish("starting", "Starting the simulation client...")
            with (self.directory / "worker.out.log").open("w", encoding="utf-8") as stdout, (self.directory / "worker.err.log").open("w", encoding="utf-8") as stderr:
                self.worker = subprocess.Popen(self.worker_command(python, client), cwd=self.args.project,
                    stdout=stdout, stderr=stderr, creationflags=FLAGS,
                    env=dict(os.environ, PYTHONUTF8="1", PYTHONUNBUFFERED="1"))
                self.status["worker_pid"] = self.worker.pid
                deadline = time.monotonic() + 15
                banner = "Jev worker listening on"
                while banner not in (self.directory / "worker.out.log").read_text(encoding="utf-8", errors="replace"):
                    self.check_cancelled()
                    if self.worker.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError("Client could not start; inspect worker.err.log (possible UDP port conflict).")
                    time.sleep(.1)
                self.publish("ready", f"{self.args.backend} ready")
                while not self.cancelled():
                    if self.worker.poll() is not None:
                        raise RuntimeError("Simulation client exited; inspect worker.err.log.")
                    self.read_worker_updates()
                    time.sleep(.1)
        except Cancelled:
            self.publish("stopping", "Stopping this session...")
        except Exception as error:
            self.publish("error", str(error))
        finally:
            terminate_owned(self.worker)
            if self.service_started and self.owns_current_service():
                try:
                    self.script(self.service_directory, "stop", self.args.backend, timeout=30, cancellable=False)
                except Exception as error:
                    self.publish("error", f"Client stopped, service cleanup failed: {error}")
            if self.status["phase"] != "error":
                self.publish("stopped", "Session stopped")
            self.owner.close()
            if self.deployment_lock:
                self.deployment_lock.close()

    def worker_command(self, python, client):
        command = [str(python), "-u", str(client), "--backend", self.args.backend,
            "--port", str(self.args.unity_port), "--listen-port", str(self.args.worker_port),
            "--log", self.status["worker_log"]]
        if self.args.backend == "cloud":
            command += ["--endpoint", CLOUD[0], "--model", CLOUD[1], "--prompt-profile", CLOUD[2]]
        return command

    def take_over_project_worker(self):
        owners = inspect_udp_owner(self.args.worker_port)
        if len(owners) != 1:
            raise RuntimeError(f"UDP {self.args.worker_port} is in use by an unrecognized client. Stop it manually first.")
        record = owners[0]
        arguments = command_arguments(record.get("command") or "")
        if (ntpath.basename(record.get("executable") or "").lower() not in ("python.exe", "pythonw.exe") or
            not project_worker_matches(arguments, self.args.project, self.args.unity_port, self.args.worker_port)):
            raise RuntimeError(f"UDP {self.args.worker_port} belongs to another process or an ambiguous relative launch. Stop that client manually first.")
        handle = ProcessHandle(int(record["pid"]))
        try:
            if not handle.alive() or str(handle.start_ticks()) != str(record["start_ticks"]):
                raise RuntimeError("Previous client identity changed. Retry model selection.")
            self.publish("starting", "Stopping this project's previous Python client...")
            subprocess.run(["taskkill", "/PID", str(record["pid"]), "/T", "/F"],
                           capture_output=True, creationflags=FLAGS, timeout=15, check=True)
            deadline = time.monotonic() + 10
            while not udp_available(self.args.worker_port):
                self.check_cancelled()
                if time.monotonic() >= deadline:
                    raise RuntimeError("Previous project client has not released its UDP port.")
                time.sleep(.1)
        finally:
            handle.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=(*BACKENDS, "cloud"), required=True)
    parser.add_argument("--ai-root", type=Path, default=Path("D:/AI"))
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    parser.add_argument("--unity-port", type=int, default=5005)
    parser.add_argument("--worker-port", type=int, default=5006)
    args = parser.parse_args()
    if os.name != "nt":
        raise SystemExit("Local model buttons currently require Windows deployments.")
    if any(not 1 <= port <= 65535 for port in (args.unity_port, args.worker_port)):
        parser.error("Ports must be 1..65535")
    Session(args).run()


if __name__ == "__main__":
    main()
