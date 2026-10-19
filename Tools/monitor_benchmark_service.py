"""Hold Windows process handles and record actual exit codes during a benchmark."""
import argparse
import ctypes
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text(encoding="utf-8-sig"))
    pids = [snapshot["manifest"]["pid"], *[p["ProcessId"] for p in snapshot["children"]]]
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.GetExitCodeProcess.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handles = {pid: kernel.OpenProcess(0x1000, False, pid) for pid in pids}
    try:
        with args.output.open("w", encoding="utf-8") as log:
            while True:
                codes = {}
                for pid, handle in handles.items():
                    code = ctypes.c_ulong()
                    codes[pid] = code.value if handle and kernel.GetExitCodeProcess(handle, ctypes.byref(code)) else None
                log.write(json.dumps({"time": time.time(), "process_exit_codes": codes,
                                      "still_active_code": 259}) + "\n")
                log.flush()
                if (args.benchmark / "summary.json").exists():
                    break
                time.sleep(5)
    finally:
        for handle in handles.values():
            if handle:
                kernel.CloseHandle(handle)


if __name__ == "__main__":
    main()
