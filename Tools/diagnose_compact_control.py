"""Experimental complete 56-action formats; diagnostics only, never sends commands to Unity."""
import argparse
import copy
import json
import math
from pathlib import Path
import random
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from boat_client import BACKENDS  # noqa: E402
from choice_control import ACTION_CHOICES, build_payload  # noqa: E402
from diagnose_model_control import observation  # noqa: E402

CASES = (
    ("ahead-rest", 0, 0, (10, 0), False),
    ("reverse-ahead", -1, 0, (10, 0), False),
    ("left", 1, 0, (8, 4), False),
    ("right", 1, 0, (8, -4), False),
    ("brake-left-yaw", 1, 20, (10, 0), False),
    ("brake-right-yaw", 1, -20, (10, 0), False),
    ("approach-parking", .1, 0, (8, 0), True),
    ("brake-parking", 2, 0, (1.5, 0), True),
    ("inside-slow", .3, 0, (0, 0), True),
    ("parked", 0, 0, (0, 0), True),
)
INSTRUCTIONS = (
    "Boat frame target=[ahead,left] in metres; v is signed m/s, w is yaw deg/s. "
    "Each option is (a,alpha): longitudinal m/s^2, angular deg/s^2. +a raises v, +alpha raises left yaw. "
    "Damping is [linear,angular] in 1/s; zero inputs permit coasting. The boat moves along its bow without side drift. "
    "Choose one pair held for hold seconds to enter radius, preferably travelling forward. "
    "If stop=true, also require abs(v)<=stop_v inside the circle. Brake before overshoot. "
    "Positive a arrests reverse travel. Steer toward target and brake yaw before alignment. "
    "These are accelerations, not target velocities."
)


def make_payload(obs, style, model):
    # Reuse production coordinate conversion; do not derive a preferred action in the input.
    state = build_payload(obs, model, "compact")["state"]
    dyn, boat, target = state["dynamics"], state["boat"], state["target"]
    small = {"target": [target["forward_m"], target["left_m"]], "v": boat["speed_mps"],
             "w": boat["yaw_rate_deg_s"], "stop": obs["target"]["requires_stop"],
             "radius": target["arrival_radius_m"], "stop_v": obs["target"]["stop_speed_mps"],
             "damping": [dyn["linear_damping"], dyn["angular_damping"]],
             "v_limits": [-dyn["max_reverse_speed"], dyn["max_forward_speed"]],
             "w_limit": dyn["max_yaw_rate_deg"], "hold": state["planning"]["action_duration_s"]}
    criteria, mapping, values = {}, {}, {}
    option_style = "tuple" if style == "goal" else "words" if style == "facts" else style
    for i, action in enumerate(ACTION_CHOICES.values()):
        a = obs["action_space"]["acceleration_fractions"][action["acceleration_index"]] * dyn["max_acceleration"]
        alpha = obs["action_space"]["turn_fractions"][action["turn_index"]] * dyn["max_angular_acceleration_deg"]
        key = f"k{i:02}"
        value = f"({a:.6g},{alpha:.6g})"
        if option_style == "named":
            value = f"a={a:.6g},alpha={alpha:.6g}"
        elif option_style == "words":
            linear = "forward accel" if a > 0 else "backward accel" if a < 0 else "zero linear input"
            turn = "left accel" if alpha > 0 else "right accel" if alpha < 0 else "zero angular input"
            value += f" {linear},{turn}"
        elif option_style != "tuple":
            raise ValueError("Unknown experimental format")
        criteria[key] = value
        mapping[key] = dict(action)
        values[key] = [a, alpha]
    assert len(criteria) == len({tuple(a.values()) for a in mapping.values()}) == 56
    instructions = INSTRUCTIONS
    if style in ("goal", "facts"):
        small.pop("stop")
        if obs["target"]["requires_stop"]:
            goal = "Final parking: enter radius AND require abs(v)<=stop_v inside the circle. "
        else:
            small.pop("stop_v")
            goal = "Checkpoint: enter radius; stopping is not required. "
        instructions = INSTRUCTIONS.replace(
            "Choose one pair held for hold seconds to enter radius, preferably travelling forward. "
            "If stop=true, also require abs(v)<=stop_v inside the circle. ",
            goal + "Choose one pair held for hold seconds, preferably travelling forward. ")
    if style == "facts":
        forward, left = small["target"]
        small.update(distance=math.hypot(forward, left), bearing_deg=math.degrees(math.atan2(left, forward)),
                     motion="forward" if small["v"] > 0 else "backward" if small["v"] < 0 else "stationary",
                     yaw_motion="left" if small["w"] > 0 else "right" if small["w"] < 0 else "none",
                     inside=math.hypot(forward, left) <= small["radius"])
        instructions += " Distance is metres; bearing_deg is target angle left of the bow, not a control instruction."
    return {"model": model, "state": small,
            "questions": {"action_0": {"type": "choice", "instructions": instructions, "criteria": criteria}}}, mapping, values


def preferred(name, action):
    a, t = action["acceleration_index"], action["turn_index"]
    return {"ahead-rest": a > 3 and t == 3, "reverse-ahead": a > 3 and t == 3,
            "left": t > 3, "right": t < 3, "brake-left-yaw": t < 3, "brake-right-yaw": t > 3,
            "approach-parking": a > 3 and t == 3, "brake-parking": a < 3 and t == 3,
            "inside-slow": a == 2 and t == 3, "parked": a == 3 and t == 3}[name]


def reordered(payload, order):
    result = copy.deepcopy(payload)
    items = list(result["questions"]["action_0"]["criteria"].items())
    if order == "reverse":
        items.reverse()
    elif order == "shuffle":
        random.Random(27).shuffle(items)
    result["questions"]["action_0"]["criteria"] = dict(items)
    return result


def tests(model, styles, suite):
    for style in styles:
        if suite != "copy":
            for name, speed, yaw, target, stop in CASES:
                obs = observation(speed, target, stop)
                obs["boat"]["yaw_rate_deg_s"] = yaw
                payload, mapping, values = make_payload(obs, style, model)
                for order in ("original", "reverse", "shuffle"):
                    yield "control", style, name, order, reordered(payload, order), mapping, values, None, obs
        # Copy tasks replace state and instructions, so goal==tuple and facts==words.
        # Avoid identical requests in a combined run; an explicitly selected format still works.
        copy_style = "tuple" if style == "goal" else "words" if style == "facts" else style
        if suite != "control" and not (copy_style != style and copy_style in styles):
            payload, mapping, values = make_payload(observation(), style, model)
            for expected in ("k03", "k24", "k52"):
                for order in ("original", "reverse", "shuffle"):
                    result = reordered(payload, order)
                    result["state"] = f"The explicitly required action label is {expected}."
                    result["questions"]["action_0"]["instructions"] = "Select exactly the option with the action label explicitly required in the state."
                    yield "copy", style, expected, order, result, mapping, values, expected, None


def summarize(rows):
    keys = dict.fromkeys((row["group"], row["style"]) for row in rows)
    out = []
    for group, style in keys:
        selected = [row for row in rows if (row["group"], row["style"]) == (group, style)]
        tokens = [row["raw_response"]["usage"]["input_tokens"] for row in selected if row["raw_response"]]
        out.append({"group": group, "style": style, "preferred": sum(row["passed"] for row in selected),
                    "total": len(selected), "errors": sum(row["error"] is not None for row in selected),
                    "token_range": [min(tokens), max(tokens)] if tokens else None,
                    "by_case": {case: sum(row["passed"] for row in selected if row["case"] == case)
                                for case in dict.fromkeys(row["case"] for row in selected)}})
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("laya", "kev"), default="laya")
    parser.add_argument("--format", choices=("all", "tuple", "named", "words", "goal", "facts"), default="all")
    parser.add_argument("--suite", choices=("all", "control", "copy"), default="all")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output path")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    endpoint, model, _ = BACKENDS[args.backend]
    styles = ("tuple", "named", "words", "goal", "facts") if args.format == "all" else (args.format,)
    report = {"backend": args.backend, "experimental_only": True, "navigation_qualified": False, "results": [], "summary": []}
    for group, style, case, order, payload, mapping, values, expected, obs in tests(model, styles, args.suite):
        if args.backend == "laya":
            payload.update(max_len=4096, head_max_len=3072)
        started = time.perf_counter()
        raw, error, action, choice, passed = None, None, None, None, False
        try:
            request = urllib.request.Request(endpoint, data=json.dumps(payload, allow_nan=False).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=60) as response:
                raw = json.load(response)
            choice = raw["answers"]["action_0"]["choice"]
            action = mapping[choice]
            passed = choice == expected if group == "copy" else preferred(case, action)
            if args.backend == "laya":
                usage = raw["usage"]
                if not usage.get("budget_verified") or usage.get("truncated") or usage["budget"][0]["options"] != 56:
                    raise ValueError("Full 56-option no-truncation guarantee missing")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            error = f"{type(exc).__name__}: {exc}"
            passed = False
        report["results"].append({"group": group, "style": style, "case": case, "order": order,
                                  "observation": obs, "payload": payload, "action_mapping": mapping,
                                  "action_values": values, "raw_response": raw, "choice": choice, "action": action,
                                  "selected_values": values.get(choice), "passed": passed, "error": error,
                                  "elapsed_s": time.perf_counter()-started})
        report["summary"] = summarize(report["results"])
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        if len(report["results"]) % 3 == 0:
            print(json.dumps({"group": group, "style": style, "case": case,
                              "preferred": sum(row["passed"] for row in report["results"][-3:]),
                              "actions": [row["action"] for row in report["results"][-3:]]}), flush=True)
        if error and error.startswith("URLError"):
            break
    print(json.dumps(report["summary"]), flush=True)
    return 2 if any(row["error"] for row in report["results"]) else 1 if any(not row["passed"] for row in report["results"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
