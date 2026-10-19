"""Trace the real local model/UDP/Unity chain in an existing independent test copy."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from boat_client import BACKENDS, IDENTITY_FIELDS, valid_request  # noqa: E402
from choice_control import build_payload, decode_action  # noqa: E402


def read_lines(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8-sig").splitlines() if line.strip()]


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def capture(args, output):
    project = args.unity_project.resolve()
    if project == ROOT or ROOT in project.parents:
        raise ValueError("Use an independent Unity copy outside the working project")
    compared = []
    for folder, pattern in (("Assets/Scripts", "*.cs"), ("Assets/StreamingAssets", "*.json"),
                            ("ProjectSettings", "TimeManager.asset")):
        for original in (ROOT / folder).rglob(pattern):
            relative = original.relative_to(ROOT)
            other = project / relative
            if not other.is_file() or digest(original) != digest(other):
                raise ValueError(f"Test copy differs: {relative}")
            compared.append({"path": str(relative), "sha256": digest(original)})
    original = (ROOT / "Assets/Scenes/Main.unity").read_text(encoding="utf-8")
    scene = (project / "Assets/Scenes/Main.unity").read_text(encoding="utf-8")
    normalized = scene.replace("  port: 15005\n", "  port: 5005\n").replace("  targetPort: 15006\n", "  targetPort: 5006\n")
    if normalized.rstrip() != original.rstrip() or "  port: 15005\n" not in scene or "  targetPort: 15006\n" not in scene:
        raise ValueError("Test scene must match Main except isolated UDP ports 15005/15006")
    save(output / "source-check.json", compared)
    for port in (15005, 15006):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            probe.bind(("127.0.0.1", port))
    helper = project / "Assets/DeploymentValidation/ModelTraceValidation.cs"
    helper.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / "Tools/ModelTraceValidation.cs", helper)
    meta = helper.with_suffix(".cs.meta")
    if not meta.exists():
        meta.write_text("fileFormatVersion: 2\nguid: 53ee852d50464cf7a03d41c5104306cd\n", encoding="utf-8")
    env = dict(os.environ, USV_TRACE_OUTPUT=str(output / "unity-trace.jsonl"), PYTHONUNBUFFERED="1", PYTHONUTF8="1")
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    with (output / "worker.out.log").open("w", encoding="utf-8") as stdout, (output / "worker.err.log").open("w", encoding="utf-8") as stderr:
        worker = subprocess.Popen([sys.executable, str(ROOT / "jev_api/boat_client.py"), "--backend", args.backend,
                                   "--port", "15005", "--listen-port", "15006", "--log", str(output / "worker.jsonl")],
                                  cwd=ROOT, env=env, stdout=stdout, stderr=stderr, creationflags=flags)
        editor = None
        try:
            time.sleep(1)
            if worker.poll() is not None:
                raise RuntimeError("Test worker exited; inspect worker.err.log")
            editor = subprocess.Popen([str(args.unity_exe), "-batchmode", "-nographics", "-projectPath", str(project),
                                       "-executeMethod", "ModelTraceValidation.Run", "--model-trace-validation",
                                       "-logFile", str(output / "unity-editor.log")], env=env, creationflags=flags)
            code = editor.wait(timeout=240)
            if code:
                raise RuntimeError(f"Unity trace failed ({code}); inspect unity-editor.log")
        finally:
            if editor is not None and editor.poll() is None:
                editor.terminate()
                editor.wait(timeout=30)
            worker.terminate()
            worker.wait(timeout=15)


def analyze(output):
    unity = read_lines(output / "unity-trace.jsonl")
    workers = [r for r in read_lines(output / "worker.jsonl") if r.get("event") == "decision"]
    initial = next(r for r in unity if r["event"] == "initial")
    steps = {r["tick"]: r for r in unity if r["event"] == "step"}
    received = [r for r in unity if r["event"] == "received"]
    by_tick = {0: initial, **steps}
    checks = []
    for row in workers:
        req, result, payload = row["request"], row["result"], row["payload"]
        raw_action = decode_action(row["raw_response"])
        obs, tick = req["observation"], req["tick"]
        before = by_tick[tick]
        match = [r for r in received if all(r.get(k) == req[k] for k in IDENTITY_FIELDS)]
        a = obs["action_space"]["acceleration_fractions"][raw_action["acceleration_index"]] * obs["dynamics"]["max_acceleration"]
        alpha = obs["action_space"]["turn_fractions"][raw_action["turn_index"]] * obs["dynamics"]["max_angular_acceleration_deg"]
        state_ok = (abs(obs["boat"]["speed_mps"] - before["speed"]) < 1e-5
                    and abs(obs["boat"]["heading_deg"] - before["heading_deg"]) < 1e-4
                    and abs(obs["boat"]["yaw_rate_deg_s"] - before["yaw_rate_deg"]) < 1e-4
                    and all(abs(x-y) < 1e-5 for x, y in zip(obs["boat"]["position_xy_m"], [before["x"], before["y"]])))
        tx, ty = obs["target"]["position_xy_m"]
        waypoint = before["mission"]["next_target"]
        target_ok = (abs(tx-waypoint["x"]) < 1e-5 and abs(ty-waypoint["y"]) < 1e-5
                     and abs(obs["target"]["arrival_radius_m"]-before["mission"]["arrival_radius_m"]) < 1e-5)
        dx, dy = tx - before["x"], ty - before["y"]
        h = math.radians(before["heading_deg"])
        target = payload["state"]["target"]
        coordinates_ok = (abs(target["forward_m"] - (dx*math.cos(h)+dy*math.sin(h))) < 1e-4
                          and abs(target["left_m"] - (-dx*math.sin(h)+dy*math.cos(h))) < 1e-4)
        physics_error = 0
        actuator_ok = True
        dt = before["parameters"]["fixed_dt"]
        count = round(req["observation"]["planning"]["action_duration_s"] / dt)
        previous = before
        for index in range(tick+1, tick+count+1):
            current = steps[index]
            p = previous["parameters"]
            if p["acceleration_time_constant"] or p["angular_acceleration_time_constant"]:
                raise ValueError("Reference check currently requires zero actuator time constants")
            speed = min(p["max_forward_speed"], max(-p["max_reverse_speed"], previous["speed"] + (a-p["linear_damping"]*previous["speed"])*dt))
            yaw = min(p["max_yaw_rate_deg"], max(-p["max_yaw_rate_deg"], previous["yaw_rate_deg"] + (alpha-p["angular_damping"]*previous["yaw_rate_deg"])*dt))
            heading = math.radians(previous["heading_deg"] + yaw*dt)
            x, y = previous["x"] + speed*math.cos(heading)*dt, previous["y"] + speed*math.sin(heading)*dt
            heading_error = abs((current["heading_deg"] - math.degrees(heading) + 180) % 360 - 180)
            physics_error = max(physics_error, abs(x-current["x"]), abs(y-current["y"]), abs(speed-current["speed"]), abs(yaw-current["yaw_rate_deg"]), heading_error)
            actuator_ok &= abs(current["actual_acceleration"]-a) < 1e-5 and abs(current["actual_angular_acceleration_deg"]-alpha) < 1e-4
            previous = current
        identity_ok = bool(match) and all(all(r.get(k) == result[k] for k in (*IDENTITY_FIELDS, "version", "type", "action", "action_duration_s")) for r in match)
        item = {"tick": tick, "choice": row["raw_response"]["answers"]["action_0"]["choice"], "action": raw_action,
                "a_mps2": a, "alpha_deg_s2": alpha, "target_boat_frame": target,
                "before": {k: before[k] for k in ("x", "y", "speed", "heading_deg", "yaw_rate_deg")},
                "after": {k: previous[k] for k in ("x", "y", "speed", "heading_deg", "yaw_rate_deg")},
                "request_valid": valid_request(req), "state_matches": state_ok, "target_matches": target_ok,
                "coordinates_match": coordinates_ok,
                "decode_matches": raw_action == result["action"], "udp_matches": identity_ok,
                "actuator_matches": actuator_ok, "max_step_error": physics_error,
                "step_physics_matches": physics_error < 0.0002, "elapsed_s": row["elapsed_s"]}
        item["chain_passed"] = all(item[k] for k in ("request_valid", "state_matches", "target_matches", "coordinates_match", "decode_matches", "udp_matches", "actuator_matches", "step_physics_matches"))
        checks.append(item)
    summary = {"backend": workers[0]["backend"], "decisions": len(checks), "steps": len(steps),
               "chain_passed": len(checks) == 12 and len(steps) == 240 and all(r["chain_passed"] for r in checks),
               "negative_acceleration_decisions": sum(r["a_mps2"] < 0 for r in checks), "checks": checks,
               "navigation_qualified": False, "visual_validation": False}
    save(output / "chain-summary.json", summary)
    print(json.dumps({k: v for k, v in summary.items() if k != "checks"}), flush=True)
    return workers


def replay(args, output, workers):
    endpoint, model, profile = BACKENDS[args.backend]
    original = workers[0]["payload"]
    cases = []
    for order in ("original", "reversed", "shuffled"):
        for repeat in range(3):
            payload = copy.deepcopy(original)
            items = list(payload["questions"]["action_0"]["criteria"].items())
            if order == "reversed":
                items.reverse()
            elif order == "shuffled":
                random.Random(27).shuffle(items)
            payload["questions"]["action_0"]["criteria"] = dict(items)
            cases.append(("repeat-order", order, repeat, payload))
    base = workers[0]["request"]["observation"]
    for name, x, y, speed in (("ahead-rest", 10, 0, 0), ("behind-rest", -10, 0, 0),
                              ("left-front", 10, 5, 0), ("right-front", 10, -5, 0),
                              ("ahead-reverse", 10, 0, -1)):
        obs = copy.deepcopy(base)
        obs["boat"].update(position_xy_m=[0, 0], heading_deg=0, speed_mps=speed, yaw_rate_deg_s=0)
        obs["target"].update(position_xy_m=[x, y], requires_stop=False)
        payload = build_payload(obs, model, profile)
        if args.backend == "laya":
            payload.update(max_len=4096, head_max_len=3072)
        for repeat in range(3):
            cases.append(("state-change", name, repeat, copy.deepcopy(payload)))
    # Explicit label matching separates a 56-option reading problem from physical reasoning.
    # These artificial requests never control Unity and are not navigation tests.
    for expected in ("positive_full__no_angular_input", "negative_full__no_angular_input", "coast__no_angular_input"):
        for repeat in range(3):
            payload = copy.deepcopy(original)
            payload["state"] = f"The explicitly required action label is {expected}."
            payload["questions"]["action_0"]["instructions"] = "Select exactly the action label explicitly required in the state."
            items = list(payload["questions"]["action_0"]["criteria"].items())
            if repeat == 1:
                items.reverse()
            elif repeat == 2:
                random.Random(27).shuffle(items)
            payload["questions"]["action_0"]["criteria"] = dict(items)
            cases.append(("explicit-label-56", expected, repeat, payload))
    if args.labels_only:
        cases = [case for case in cases if case[0] == "explicit-label-56"]
    report_name = "label-check" if args.labels_only else "replay"
    results = []
    for group, case, repeat, payload in cases:
        started = time.perf_counter()
        req = urllib.request.Request(endpoint, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as response:
            raw = json.load(response)
        results.append({"group": group, "case": case, "repeat": repeat, "elapsed_s": time.perf_counter()-started,
                        "action": decode_action(raw), "payload": payload, "raw_response": raw,
                        "explicit_label_matches": raw["answers"]["action_0"]["choice"] == case if group == "explicit-label-56" else None})
        save(output / f"{report_name}.json", results)
    groups = []
    for group, case in dict.fromkeys((r["group"], r["case"]) for r in results):
        rows = [r for r in results if (r["group"], r["case"]) == (group, case)]
        groups.append({"group": group, "case": case, "actions": [r["action"] for r in rows],
                       "stable": len({json.dumps(r["action"], sort_keys=True) for r in rows}) == 1,
                       "explicit_label_passed": sum(r["explicit_label_matches"] for r in rows) if group == "explicit-label-56" else None})
    save(output / f"{report_name}-summary.json", groups)
    for row in groups:
        print(json.dumps(row), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("laya", "kev"), default="laya")
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--labels-only", action="store_true", help="Only run explicit-label diagnostics during replay")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--unity-project", type=Path, default=Path("D:/AI/Laya/validation-project"))
    parser.add_argument("--unity-exe", type=Path, default=Path("D:/Unity/6000.6.5f1/Editor/Unity.exe"))
    args = parser.parse_args()
    if args.labels_only and not args.replay:
        parser.error("--labels-only requires --replay")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.capture:
        if (output / "unity-trace.jsonl").exists() or (output / "worker.jsonl").exists():
            parser.error("Use a new output folder; existing traces must not be overwritten")
        capture(args, output)
    workers = analyze(output)
    if workers[0]["backend"] != args.backend:
        parser.error("Selected backend differs from captured trace")
    if args.replay:
        replay(args, output, workers)
    return 0 if json.loads((output / "chain-summary.json").read_text(encoding="utf-8"))["chain_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
