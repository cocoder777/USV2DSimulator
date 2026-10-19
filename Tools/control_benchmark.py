"""Control benchmark scenarios, metrics and Unity/worker orchestration."""
import argparse
import json
import math
import os
import shutil
import socket
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
DEFAULT_SCENARIOS = TOOLS / "scenarios" / "control_benchmark_scenarios.json"


def distance_to_segment(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def distance_to_polyline(point, polyline):
    if not polyline:
        raise ValueError("Reference polyline is empty")
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    return min(distance_to_segment(point, polyline[index], polyline[index + 1])
               for index in range(len(polyline) - 1))


def route_polyline(route):
    samples = (route.get("path") or {}).get("samples")
    if samples:
        return [(float(sample["x"]), float(sample["y"])) for sample in samples]
    return [(float(point["x"]), float(point["y"])) for point in route["waypoints"]]


def validate_scenario(scenario):
    if not isinstance(scenario, dict) or not isinstance(scenario.get("name"), str) or not scenario["name"]:
        raise ValueError("Scenario needs a non-empty name")
    route = scenario.get("route")
    if not isinstance(route, dict):
        raise ValueError("Scenario needs a route")
    waypoints = route.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("Route needs at least two waypoints")
    for index, point in enumerate(waypoints):
        if not isinstance(point, dict) or point.get("role") != ("start" if index == 0 else "end" if index == len(waypoints) - 1 else "checkpoint"):
            raise ValueError("Waypoint roles must be start, checkpoints, end")
        for key in ("x", "y"):
            value = point.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("Waypoint coordinates must be finite")
    if "path" in route:
        samples = route["path"].get("samples")
        if not isinstance(samples, list) or len(samples) < 2:
            raise ValueError("Curve path needs at least two samples")
    duration = scenario.get("max_simulation_seconds")
    if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration <= 0:
        raise ValueError("Scenario needs positive max_simulation_seconds")
    passed = scenario.get("stop_after_passed")
    if passed is not None and (not isinstance(passed, int) or not 0 <= passed < len(waypoints)):
        raise ValueError("stop_after_passed must be a valid waypoint count")
    return scenario


def load_scenarios(path=DEFAULT_SCENARIOS):
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    scenarios = document.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("Scenario file needs a non-empty scenarios list")
    names = set()
    for scenario in scenarios:
        validate_scenario(scenario)
        if scenario["name"] in names:
            raise ValueError("Duplicate scenario name: " + scenario["name"])
        names.add(scenario["name"])
    return scenarios


def _rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def summarize_run(scenario, steps, result):
    steps = [step for step in steps if step.get("event") == "step"]
    if not steps:
        reason = result.get("reason", "no_steps") if result else "no_steps"
        return {"scenario": scenario["name"], "completed": False, "end_reason": reason,
                "failure_reason": reason, "steps": 0, "decisions": 0,
                "outcome": classify_outcome(result),
                "simulation_time_s": 0.0, "final_distance_m": None, "final_speed_mps": None,
                "stop_speed_excess_mps": None, "max_path_deviation_m": None,
                "rms_path_deviation_m": None, "mean_path_deviation_m": None, "mean_abs_heading_error_deg": None,
                "rms_heading_error_deg": None, "max_abs_heading_error_deg": None,
                "max_abs_yaw_rate_deg_s": None, "mean_abs_acceleration_mps2": None,
                "mean_abs_angular_acceleration_deg_s2": None, "action_switches": 0,
                "target_distance_start_m": None, "target_distance_min_m": None}
    polyline = route_polyline(scenario["route"])
    deviations = [distance_to_polyline((step["x"], step["y"]), polyline) for step in steps]
    heading_errors = [abs((step.get("bearing_deg", 0.0) + 180.0) % 360.0 - 180.0) for step in steps]
    target_distances = [float(step.get("target_distance", 0.0)) for step in steps]
    final = result if result and "x" in result else steps[-1]
    target = scenario["route"]
    stop_speed = float(target.get("stop_speed_mps", 0.0))
    final_speed = float(final.get("speed", 0.0))
    stop_after = scenario.get("stop_after_passed")
    final_waypoint = stop_after is None
    stop_excess = (max(0.0, abs(final_speed) - stop_speed)
                   if stop_speed > 0 and final_waypoint else 0.0)
    # Mission.Next is null on completion and already advances after checkpoint contact.
    # Measure the scenario's actual goal from world coordinates instead of that field.
    goal = target["waypoints"][stop_after - 1 if stop_after is not None else -1]
    final_distance = math.hypot(final["x"] - goal["x"], final["y"] - goal["y"])
    switches = 0
    previous_action = None
    for step in steps:
        action = (step.get("acceleration_index"), step.get("turn_index"))
        if action != (None, None):
            if previous_action is not None and action != previous_action:
                switches += 1
            previous_action = action
    completed = bool(result.get("completed")) if result else False
    end_reason = result.get("reason", "completed" if completed else "failed") if result else "no_result"
    return {
        "scenario": scenario["name"],
        "completed": completed,
        "outcome": classify_outcome(result),
        "end_reason": end_reason,
        "failure_reason": "completed" if completed else end_reason,
        "steps": len(steps),
        "decisions": int(result.get("decisions", 0)) if result else 0,
        "simulation_time_s": float(result.get("simulation_time", final.get("simulation_time", 0.0))) if result else float(final.get("simulation_time", 0.0)),
        "final_x": float(final["x"]),
        "final_y": float(final["y"]),
        "final_distance_m": final_distance,
        "current_target_distance_m": (float(final["target_distance"])
            if final.get("target_index", 0) < len(target["waypoints"]) and "target_distance" in final else None),
        "final_speed_mps": final_speed,
        "parking_speed_mps": abs(final_speed) if stop_after is None else None,
        "stop_speed_excess_mps": stop_excess,
        "passed_count": int(result.get("passed_count", final.get("passed_count", 0))) if result else int(final.get("passed_count", 0)),
        "max_path_deviation_m": max(deviations),
        "mean_path_deviation_m": statistics.mean(deviations),
        "rms_path_deviation_m": _rms(deviations),
        "mean_abs_heading_error_deg": sum(heading_errors) / len(heading_errors),
        "rms_heading_error_deg": _rms(heading_errors),
        "max_abs_heading_error_deg": max(heading_errors),
        "max_abs_yaw_rate_deg_s": max(abs(float(step.get("yaw_rate_deg", 0.0))) for step in steps),
        "mean_abs_acceleration_mps2": sum(abs(float(step.get("acceleration", 0.0))) for step in steps) / len(steps),
        "mean_abs_angular_acceleration_deg_s2": sum(abs(float(step.get("angular_acceleration_deg", 0.0))) for step in steps) / len(steps),
        "action_switches": switches,
        "target_distance_start_m": target_distances[0],
        "target_distance_min_m": min(target_distances),
    }


def classify_outcome(result):
    if not result:
        return "incomplete"
    if result.get("completed"):
        return "success"
    reason = result.get("reason", "")
    if reason == "timeout":
        return "navigation_failure"  # Full simulation budget, not an HTTP timeout.
    if "timeout" in reason.lower() or "timed out" in reason.lower():
        return "technical_timeout"
    if "10061" in reason or "service unavailable" in reason.lower():
        return "service_interruption"
    if reason.startswith("runner_failed"):
        return "response_or_service_error"
    return "incomplete"


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def _scenario_name(record):
    label = record.get("scenario")
    if isinstance(label, dict):
        return label.get("name")
    return label


def scenario_trace(trace, name):
    started = False
    selected = []
    result = None
    for record in trace:
        if record.get("event") == "scenario_start" and _scenario_name(record) == name:
            started = True
            selected = [record]
            result = None
        elif started:
            selected.append(record)
            if record.get("event") == "scenario_end":
                result = record
                break
    return selected, result


def summarize_trace(scenario, trace):
    records, result = scenario_trace(trace, scenario["name"])
    return summarize_run(scenario, records, result)


def write_summary(output, scenarios, trace, controller=None, method=None):
    summaries = [summarize_trace(scenario, trace) for scenario in scenarios]
    worker_path = Path(output) / "worker.jsonl"
    workers = read_jsonl(worker_path) if worker_path.exists() else []
    for scenario, summary in zip(scenarios, summaries):
        records, _ = scenario_trace(trace, scenario["name"])
        ids = {record.get("decision_id") for record in records if record.get("event") in ("request", "decision", "decision_error")}
        matched = [row for row in workers if row.get("event") == "decision" and row.get("request", {}).get("decision_id") in ids]
        latencies = sorted(float(row["elapsed_s"]) for row in matched)
        summary["decision_latency_s"] = ({"count": len(latencies), "mean": statistics.mean(latencies),
            "median": statistics.median(latencies), "p95": latencies[math.ceil(.95 * len(latencies))-1], "max": max(latencies)} if latencies else None)
        summary["valid_responses"] = sum(row.get("result", {}).get("type") == "decision_result" for row in matched)
        summary["error_responses"] = sum(row.get("result", {}).get("type") == "decision_error" for row in matched)
        actions = {}
        for row in matched:
            action = row.get("result", {}).get("action")
            if action:
                key = f'{action["acceleration_index"]},{action["turn_index"]}'
                actions[key] = actions.get(key, 0) + 1
        summary["action_counts"] = actions
    document = {"controller": controller, "method": method, "scenarios": summaries}
    (Path(output) / "summary.json").write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    return summaries


def port_is_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def copy_unity_helper(project):
    helper = Path(project) / "Assets" / "DeploymentValidation" / "ControlBenchmarkValidation.cs"
    helper.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(TOOLS / "ControlBenchmarkValidation.cs", helper)
    meta = helper.with_suffix(".cs.meta")
    if not meta.exists():
        meta.write_text("fileFormatVersion: 2\nguid: 6f0e4d2a9c314f2e8d7b1a5c3e9f2d41\n", encoding="utf-8")


def worker_command(controller, method, log):
    if controller in ("laya", "kev"):
        return [sys.executable, str(ROOT / "jev_api" / "boat_client.py"),
                "--backend", controller, "--port", "15005", "--listen-port", "15006", "--log", str(log)]
    return [sys.executable, str(TOOLS / "classic_control.py"), "--method", method,
            "--port", "15005", "--listen-port", "15006", "--log", str(log)]


def check_service(controller):
    endpoints = {"laya": 8010, "kev": 8009}
    if controller not in endpoints:
        return
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(("127.0.0.1", endpoints[controller])) != 0:
            raise RuntimeError(f"{controller} service is not listening on {endpoints[controller]}")


def run(args):
    scenarios = load_scenarios(args.scenarios)
    if args.scenario:
        scenarios = [scenario for scenario in scenarios if scenario["name"] in args.scenario]
        if not scenarios:
            raise ValueError("No selected scenario")
    for port in (15005, 15006):
        if not port_is_free(port):
            raise RuntimeError(f"UDP port {port} is already in use")
    check_service(args.controller)
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError("Output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    project = Path(args.unity_project).resolve()
    if project == ROOT or ROOT in project.parents:
        raise RuntimeError("Unity must run from an independent copy")
    copy_unity_helper(project)
    trace_path = output / "unity-trace.jsonl"
    scenario_path = output / "scenarios.json"
    scenario_path.write_text(json.dumps({"scenarios": scenarios}, ensure_ascii=False, indent=2), encoding="utf-8")
    log = output / "worker.jsonl"
    command = worker_command(args.controller, args.method, log)
    env = dict(os.environ, USV_BENCH_OUTPUT=str(trace_path),
               USV_BENCH_SCENARIOS=str(scenario_path), PYTHONUNBUFFERED="1", PYTHONUTF8="1")
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    with (output / "worker.out.log").open("w", encoding="utf-8") as stdout, (output / "worker.err.log").open("w", encoding="utf-8") as stderr:
        worker = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, creationflags=flags)
        editor = None
        editor_code = None
        try:
            time.sleep(1)
            if worker.poll() is not None:
                raise RuntimeError("Worker exited; inspect worker.err.log")
            editor = subprocess.Popen([str(args.unity_exe), "-batchmode", "-nographics",
                                       "-projectPath", str(project), "-executeMethod", "ControlBenchmarkValidation.Run",
                                       "--control-benchmark", "-logFile", str(output / "unity-editor.log")],
                                      env=env, creationflags=flags)
            try:
                deadline = time.monotonic() + args.timeout
                with (output / "gpu-telemetry.jsonl").open("w", encoding="utf-8") as telemetry:
                    while editor.poll() is None and time.monotonic() < deadline:
                        try:
                            gpu = subprocess.run(["nvidia-smi", "--query-gpu=timestamp,memory.used,memory.total,utilization.gpu", "--format=csv,noheader,nounits"],
                                capture_output=True, text=True, timeout=5, creationflags=flags)
                            port = {"laya": 8010, "kev": 8009}.get(args.controller)
                            with socket.socket() as probe:
                                probe.settimeout(.5)
                                listening = probe.connect_ex(("127.0.0.1", port)) == 0 if port else None
                            telemetry.write(json.dumps({"time": time.time(), "gpu": gpu.stdout.strip(), "listening": listening,
                                "worker_exit": worker.poll(), "editor_pid": editor.pid}) + "\n")
                            telemetry.flush()
                        except (OSError, subprocess.TimeoutExpired) as error:
                            telemetry.write(json.dumps({"time": time.time(), "error": str(error)}) + "\n")
                        time.sleep(5)
                    editor_code = editor.poll()
                    if editor_code is None:
                        editor_code = "orchestrator_timeout"
            except subprocess.TimeoutExpired:
                editor_code = "orchestrator_timeout"
        finally:
            if editor is not None and editor.poll() is None:
                editor.terminate()
                editor.wait(timeout=30)
            worker.terminate()
            worker.wait(timeout=15)
    if not trace_path.exists():
        raise RuntimeError("Unity did not write a trace")
    trace = read_jsonl(trace_path)
    summaries = write_summary(output, scenarios, trace, args.controller,
                              args.method if args.controller == "classic" else None)
    print(json.dumps({"output": str(output), "unity_exit": editor_code, "summaries": summaries}, ensure_ascii=False, indent=2))
    return 0 if all(row.get("completed") for row in summaries) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("classic", "laya", "kev"), default="classic")
    parser.add_argument("--method", choices=("pd", "lookahead"), default="lookahead")
    parser.add_argument("--scenario", action="append", help="Scenario name; repeat to select several")
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--unity-project", type=Path, default=Path("D:/AI/Laya/validation-project"))
    parser.add_argument("--unity-exe", type=Path, default=Path("D:/Unity/6000.6.5f1/Editor/Unity.exe"))
    parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
