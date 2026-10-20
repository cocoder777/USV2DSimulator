"""Run one real Unity/Jev episode and retain all requests, source and trajectory metrics."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
UNITY = shutil.which("unity")


def command(name, *args):
    p = subprocess.run(
        [
            UNITY,
            "command",
            name,
            "--caller",
            "plugin",
            "--skill",
            "unity-cli",
            "--format",
            "json",
            *args,
        ],
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        cwd=ROOT,
        timeout=60,
    )
    d = json.loads(p.stdout)
    if not d.get("success"):
        raise RuntimeError(d)
    r = d["data"]["result"]
    if isinstance(r, dict) and r.get("success") is False:
        raise RuntimeError(r)
    return r


def evaluate(code):
    return command("eval", code)["result"]


def records(path):
    if not path.exists():
        return []
    lines = path.read_text(encoding="utf-8").splitlines()
    result = []
    for line in lines:
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # A concurrently written last record may be incomplete.
    return result


def trajectory_metrics(document):
    points = document["route"]["path"]["samples"]
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.hypot(b["x"] - a["x"], b["y"] - a["y"]))

    def project(p, low=0, high=None):
        best = (float("inf"), 0)
        for i, (a, b) in enumerate(zip(points, points[1:])):
            if i < low or (high is not None and i >= high):
                continue
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            size = dx * dx + dy * dy
            t = (
                max(0, min(1, ((p["x"] - a["x"]) * dx + (p["y"] - a["y"]) * dy) / size))
                if size
                else 0
            )
            error = math.hypot(p["x"] - a["x"] - t * dx, p["y"] - a["y"] - t * dy)
            if error < best[0]:
                best = (error, lengths[i] + t * math.sqrt(size))
        return best

    radius = document["route"]["arrival_radius_m"]
    waypoints = document["route"]["waypoints"]
    indices = document["route"]["path"].get("waypoint_sample_indices") or [
        min(
            range(len(points)),
            key=lambda i: math.hypot(points[i]["x"] - p["x"], points[i]["y"] - p["y"]),
        )
        for p in waypoints
    ]
    tangents = []
    for n, i in enumerate(indices):
        # Incoming direction, before a hairpin; the tangent at the vertex can
        # already point OUT of the checkpoint and falsely mark the approach missed.
        j = i
        while j > (indices[n - 1] if n else 0) and lengths[i] - lengths[j] < radius:
            j -= 1
        a, b = points[j], points[i]
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        length = math.hypot(dx, dy)
        tangents.append((dx / length, dy / length) if length else (1, 0))
    errors = []
    active_errors = []
    missed = set()
    approached = set()
    next_index = 0
    previous = document["samples"][0]
    final_inside = False
    parking_overshoots = 0
    for p in document["samples"]:
        error, s = project(p)
        errors.append(error)

        def distance_to_step(center):
            dx, dy = p["x"] - previous["x"], p["y"] - previous["y"]
            size = dx * dx + dy * dy
            t = (
                max(
                    0,
                    min(
                        1,
                        (
                            (center["x"] - previous["x"]) * dx
                            + (center["y"] - previous["y"]) * dy
                        )
                        / size,
                    ),
                )
                if size
                else 0
            )
            return math.hypot(
                center["x"] - previous["x"] - t * dx,
                center["y"] - previous["y"] - t * dy,
            )

        while (
            next_index < len(waypoints) - 1
            and distance_to_step(waypoints[next_index]) <= radius
        ):
            next_index += 1
        if next_index == len(waypoints) - 1:
            inside = (
                math.hypot(p["x"] - waypoints[-1]["x"], p["y"] - waypoints[-1]["y"])
                <= radius
            )
            if final_inside and not inside:
                parking_overshoots += 1
            final_inside = inside
            if inside and abs(p["speed"]) <= document["route"]["stop_speed_mps"]:
                next_index += 1
        if next_index < len(waypoints):
            point = waypoints[next_index]
            if distance_to_step(point) <= 2 * radius:
                approached.add(next_index)
            tx, ty = tangents[next_index]
            if (
                next_index in approached
                and (p["x"] - point["x"]) * tx + (p["y"] - point["y"]) * ty > radius
            ):
                missed.add(point["id"])
        active = min(max(1, next_index), len(waypoints) - 1)
        start_s = max(0, lengths[indices[active - 1]] - 2 * radius)
        low = max(0, next((i for i, s in enumerate(lengths) if s >= start_s), 0) - 1)
        active_errors.append(project(p, low, indices[active])[0])
        previous = p
    return {
        "simulation_time_s": document["simulation_time"],
        "rms_error_m": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "max_error_m": max(errors),
        "mean_error_m": sum(errors) / len(errors),
        "active_leg_rms_error_m": math.sqrt(
            sum(e * e for e in active_errors) / len(active_errors)
        ),
        "active_leg_max_error_m": max(active_errors),
        "missed_points": sorted(missed),
        "parking_overshoots": parking_overshoots,
        "passed_count": document["passed_count"],
        "final_speed_mps": document["final_speed_mps"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name")
    parser.add_argument("--level", type=int, default=0)
    parser.add_argument("--max-decisions", type=int, default=160)
    parser.add_argument("--wall-timeout", type=float, default=420)
    parser.add_argument("--tracking-preference-m", type=float, default=0.25)
    args = parser.parse_args()
    folder = ROOT / "Logs/prompt_search" / args.name
    folder.mkdir(parents=True, exist_ok=False)
    for f in ("choice_control.py", "route_context.py", "tools/navigation_motion.py"):
        (folder / f).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "jev_api" / f, folder / f)
    initial = evaluate(
        "var b=UnityEngine.Object.FindFirstObjectByType<BoatUdpBridge>();b.GetComponent<SimulationRunner>().StopJev();return Newtonsoft.Json.JsonConvert.SerializeObject(new {b.port,b.targetPort,background=UnityEngine.Application.runInBackground});"
    )
    original = json.loads(initial)
    (folder / "original.json").write_text(initial, encoding="utf-8")
    evaluate(
        "var b=UnityEngine.Object.FindFirstObjectByType<BoatUdpBridge>();b.enabled=false;b.port=15015;b.targetPort=15016;b.enabled=true;UnityEngine.Application.runInBackground=true;return true;"
    )
    worker = None
    summary = {}
    path = folder / "requests.jsonl"
    started = time.monotonic()
    try:
        with (folder / "stdout.log").open("w", encoding="utf-8") as out:
            worker = subprocess.Popen(
                [
                    sys.executable,
                    "jev_api/boat_client.py",
                    "--log",
                    str(path),
                    "--exit-on-complete",
                    "--port",
                    "15015",
                    "--listen-port",
                    "15016",
                    "--tracking-preference-m",
                    str(args.tracking_preference_m),
                ],
                cwd=ROOT,
                stdout=out,
                stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                env={**os.environ, "PYTHONIOENCODING": "utf-8"},
            )
            for _ in range(200):
                if "Jev worker listening" in (folder / "stdout.log").read_text(
                    encoding="utf-8"
                ):
                    break
                if worker.poll() is not None:
                    raise RuntimeError("Worker startup failed")
                time.sleep(0.1)
            print(
                evaluate(
                    f"var b=UnityEngine.Object.FindFirstObjectByType<BoatDynamics2D>();b.GetComponent<MissionManager>().SelectMapLevel({args.level});b.maxForwardSpeed=5;b.maxReverseSpeed=1;b.speedResponseTime=0.5f;b.headingResponseTime=0.5f;b.SetMode(BoatDynamics2D.ControlMode.Jev);b.GetComponent<SimulationRunner>().actionDuration=0.4f;b.GetComponent<SimulationRunner>().StartJev();return new {{b.Tick,b.GetComponent<BoatTrajectoryRecorder>().SampleCount}};"
                ),
                flush=True,
            )
            retries = 0
            last_print = 0
            abort = None
            while worker.poll() is None:
                rows = records(path)
                ds = [r for r in rows if r.get("event") == "decision"]
                if any(r.get("event") == "completed" for r in rows):
                    break
                if ds and ds[-1]["result"]["type"] == "decision_error":
                    if retries >= 1:
                        abort = "repeated API error"
                        break
                    retries += 1
                    print(
                        "Retrying failed decision once; error remains in metrics",
                        flush=True,
                    )
                    evaluate(
                        "UnityEngine.Object.FindFirstObjectByType<SimulationRunner>().Retry();return true;"
                    )
                    time.sleep(1)
                if len(ds) > args.max_decisions:
                    abort = "decision limit"
                    break
                if time.monotonic() - started > args.wall_timeout:
                    abort = "wall timeout"
                    break
                if (
                    ds
                    and ds[-1]["payload"]["state"]
                    .get("route", {})
                    .get("cross_track_distance_m", 0)
                    > 5
                ):
                    abort = "deviation guard"
                    break
                if len(ds) > last_print + 15:
                    print(
                        json.dumps(
                            {
                                "name": args.name,
                                "decisions": len(ds),
                                "tick": ds[-1]["request"]["tick"],
                            }
                        ),
                        flush=True,
                    )
                    last_print = len(ds)
                time.sleep(1)
            state = json.loads(
                evaluate(
                    "var b=UnityEngine.Object.FindFirstObjectByType<BoatDynamics2D>();var t=b.GetComponent<BoatTrajectoryRecorder>();return Newtonsoft.Json.JsonConvert.SerializeObject(new {state=b.GetComponent<SimulationRunner>().State.ToString(),b.Tick,b.SimulationTime,b.Speed,passed=b.GetComponent<MissionManager>().Progress.NextIndex,t.LastDataPath,t.LastImagePath,t.ExportError});"
                )
            )
            (folder / "final_state.json").write_text(
                json.dumps(state, indent=2), encoding="utf-8"
            )
            rows = records(path)
            ds = [r for r in rows if r.get("event") == "decision"]
            summary = {
                "name": args.name,
                "level": args.level,
                "tracking_preference_m": args.tracking_preference_m,
                "state": state["state"],
                "abort": abort,
                "decisions": len(ds),
                "errors": [
                    r["result"]["error"]
                    for r in ds
                    if r["result"]["type"] == "decision_error"
                ],
                "resolved_models": sorted(
                    set(
                        r["raw_response"].get("model", "")
                        for r in ds
                        if isinstance(r.get("raw_response"), dict)
                    )
                ),
                "wall_time_s": time.monotonic() - started,
                "api_wait_s": sum(r["timings"].get("api_total_s", 0) for r in ds),
                "mean_input_tokens": sum(
                    r["raw_response"].get("usage", {}).get("input_tokens", 0)
                    for r in ds
                    if isinstance(r.get("raw_response"), dict)
                )
                / max(1, len(ds)),
                "source_sha256": hashlib.sha256(
                    (folder / "choice_control.py").read_bytes()
                ).hexdigest(),
            }
            if state["state"] == "Completed" and state["LastDataPath"]:
                document = json.loads(
                    Path(state["LastDataPath"]).read_text(encoding="utf-8-sig")
                )
                shutil.copy2(state["LastDataPath"], folder / "trajectory.json")
                summary.update(trajectory_metrics(document))
                summary["trajectory_plot"] = state["LastImagePath"]
            else:
                summary["simulation_time_s"] = state["SimulationTime"]
            (folder / "summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            print(json.dumps(summary, ensure_ascii=False), flush=True)
    finally:
        evaluate(
            f'var b=UnityEngine.Object.FindFirstObjectByType<BoatUdpBridge>();b.GetComponent<SimulationRunner>().StopJev();b.enabled=false;b.port={original["port"]};b.targetPort={original["targetPort"]};b.enabled=true;UnityEngine.Application.runInBackground={str(original["background"]).lower()};return true;'
        )
        if worker and worker.poll() is None:
            worker.terminate()
            worker.wait(timeout=10)


if __name__ == "__main__":
    main()
