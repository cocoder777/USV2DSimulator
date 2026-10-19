"""Local-only decision diagnostics. Exit 1 means incorrect decisions, 2 means API errors."""
import argparse
import copy
import itertools
import json
from pathlib import Path
import random
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from choice_control import build_payload  # noqa: E402

CASES = (
    ("ahead-rest", 0, [10, 0], False),
    ("reverse-ahead", -1, [10, 0], False),
    ("left", 1, [8, 4], False),
    ("right", 1, [8, -4], False),
    ("brake-parking", 3, [0.5, 0], True),
    ("parked", 0, [0, 0], True),
)
LINEAR = (
    "Full backward thrust", "Medium backward thrust", "Gentle backward thrust", "No thrust; coast",
    "Gentle forward thrust", "Medium forward thrust", "Strong forward thrust", "Full forward thrust",
)
TURN = (
    "Strong right angular acceleration", "Medium right angular acceleration", "Gentle right angular acceleration",
    "No angular acceleration", "Gentle left angular acceleration", "Medium left angular acceleration",
    "Strong left angular acceleration",
)


def observation(speed=0, target=(10, 0), stop=False):
    fixture = json.loads((ROOT / "jev_api/examples/choice56_request.json").read_text(encoding="utf-8"))["state"]
    return {
        "coordinates": fixture["coordinates"], "planning": fixture["planning"],
        "boat": {"position_xy_m": [0, 0], "heading_deg": 0, "speed_mps": speed, "yaw_rate_deg_s": 0},
        "target": {"position_xy_m": list(target), "arrival_radius_m": 1,
                   "requires_stop": stop, "stop_speed_mps": 0.1},
        "dynamics": fixture["dynamics"], "motion": fixture["motion"],
        "action_space": {"acceleration_fractions": [-1, -2/3, -1/3, 0, .25, .5, .75, 1],
                         "turn_fractions": [-1, -.5, -.25, 0, .25, .5, 1]},
    }


def baseline():
    for word in ("forward", "backward", "stop"):
        for order in itertools.permutations(("forward", "backward", "stop")):
            yield "direction-word", f"{word}/{order}", {
                "state": f"The required direction is {word}.",
                "questions": {"action_0": {"type": "choice", "instructions": "Which direction is explicitly required?",
                                            "criteria": {k: k for k in order}}},
            }, word
    descriptions = {"A": "Move forward.", "B": "Move backward.", "C": "Remain stopped."}
    for word, expected in (("forward", "A"), ("backward", "B"), ("stopped", "C")):
        for order in itertools.permutations(descriptions):
            yield "direction-neutral", f"{word}/{order}", {
                "state": f"The boat must move {word}.",
                "questions": {"action_0": {"type": "choice", "instructions": "Select the matching action.",
                                            "criteria": {k: descriptions[k] for k in order}}},
            }, expected
    for name, state, expected in (
        ("ahead-rest", "The boat is stopped. The current checkpoint is 10 metres straight ahead. Reach it by travelling forward.", "A"),
        ("reverse-ahead", "The boat is moving backward at 1 metre per second. The checkpoint is 10 metres ahead. Reach it by travelling forward.", "A"),
        ("braking", "The boat is travelling forward at 3 metres per second. The final target is only 0.5 metres ahead. Stop inside the target.", "B"),
    ):
        choices = {"A": "Apply forward acceleration to increase forward speed or reduce backward speed.",
                   "B": "Apply backward acceleration to reduce forward speed or increase backward speed.",
                   "C": "Apply zero acceleration and coast with damping."}
        for order in itertools.permutations(choices):
            yield "boat-three", f"{name}/{order}", {"state": state, "questions": {
                "action_0": {"type": "choice", "instructions": "Choose the acceleration that best reaches the target.",
                             "criteria": {k: choices[k] for k in order}}}}, expected
    base = build_payload(observation(), profile="compact")
    keys = ("negative_full__no_angular_input", "coast__no_angular_input", "positive_full__no_angular_input")
    for order in itertools.permutations(keys):
        p = copy.deepcopy(base)
        p["questions"]["action_0"]["criteria"] = {k: base["questions"]["action_0"]["criteria"][k] for k in order}
        yield "numeric-three", str(order), p, keys[2]


def formats():
    for variant in ("joint-spatial", "split-json", "split-text", "split-spatial", "coarse-spatial"):
        for name, speed, target, stop in CASES:
            obs = observation(speed, target, stop)
            state = build_payload(obs, profile="compact")["state"]
            goal = "Stop inside the final target circle." if stop else "Enter the checkpoint without needing to stop."
            if variant != "split-json":
                state = (f"{goal} Target: {target[0]:g} metres forward, {target[1]:g} metres left, radius 1 metre. "
                         f"Speed {speed:g} m/s (positive forward, negative backward); yaw rate zero. Action lasts 0.4 seconds. "
                         f"Linear damping {obs['dynamics']['linear_damping']:g}/s; angular damping {obs['dynamics']['angular_damping']:g}/s.")
                if stop:
                    state += " Stop speed must be at most 0.1 m/s inside the circle."
            if variant.endswith("spatial"):
                side = "left" if target[1] > 0 else "right" if target[1] < 0 else "straight along the bow axis"
                state += f" The target is {'ahead' if target[0] > 0 else 'alongside'} {side}."
            linear = {f"linear_{i}": f"{label}; a={obs['action_space']['acceleration_fractions'][i] * obs['dynamics']['max_acceleration']:.4g} m/s^2."
                      for i, label in enumerate(LINEAR)}
            turns = {f"turn_{i}": f"{label}; alpha={obs['action_space']['turn_fractions'][i] * obs['dynamics']['max_angular_acceleration_deg']:.4g} deg/s^2."
                     for i, label in enumerate(TURN)}
            if variant.startswith("coarse"):
                linear = {k: linear[k] for k in ("linear_0", "linear_3", "linear_7")}
                turns = {k: turns[k] for k in ("turn_0", "turn_3", "turn_6")}
            thrust = (goal + " Prefer forward travel. Forward thrust increases signed speed and arrests reverse motion. "
                      "Backward thrust brakes forward motion. Zero input permits damping. Choose thrust for the next interval.")
            steer = ("Align the bow with the target and brake yaw before alignment. Left acceleration increases left yaw; "
                     "right acceleration increases right yaw. Zero input does not instantly stop rotation.")
            if variant.startswith("joint"):
                questions = {"action_0": {"type": "choice", "instructions": thrust + " " + steer, "criteria": {
                    f"action_{a}_{t}": f"{LINEAR[a]}; {TURN[t]}." for a in range(8) for t in range(7)}}}
            else:
                questions = {"acceleration": {"type": "choice", "instructions": thrust, "criteria": linear},
                             "steering": {"type": "choice", "instructions": steer, "criteria": turns}}
            for seed in (0, 1, 27):
                q = copy.deepcopy(questions)
                for question in q.values():
                    items = list(question["criteria"].items())
                    if seed == 1:
                        items.reverse()
                    elif seed:
                        random.Random(seed).shuffle(items)
                    question["criteria"] = dict(items)
                yield variant, f"{name}/seed={seed}", {"state": state, "questions": q}, name


def correct(expected, answers):
    if "acceleration" in answers:
        a = int(answers["acceleration"]["choice"].split("_")[-1])
        t = int(answers["steering"]["choice"].split("_")[-1])
    elif answers["action_0"]["choice"].startswith("action_"):
        a, t = map(int, answers["action_0"]["choice"].split("_")[1:])
    else:
        return answers["action_0"]["choice"] == expected
    return {"ahead-rest": a > 3 and t == 3, "reverse-ahead": a > 3 and t == 3,
            "left": t > 3, "right": t < 3, "brake-parking": a < 3 and t == 3,
            "parked": a == 3 and t == 3}[expected]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("laya", "kev"), default="laya")
    parser.add_argument("--suite", choices=("baseline", "formats", "all"), default="baseline")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    endpoint, model = (("http://127.0.0.1:8010/v1/systemone", "multilingual") if args.backend == "laya"
                       else ("http://127.0.0.1:8009/v1/systemone", "kev-latest"))
    output = args.output or ROOT / "Logs" / f"diagnostic-{args.backend}-{time.strftime('%Y%m%d-%H%M%S')}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    tests = list(baseline()) if args.suite != "formats" else []
    if args.suite != "baseline":
        tests.extend(formats())
    report = {"backend": args.backend, "suite": args.suite, "results": [], "summary": []}
    for group, tag, payload, expected in tests:
        payload["model"] = model
        if args.backend == "laya":
            payload.update(max_len=4096, head_max_len=3072)
        started = time.perf_counter()
        error = None
        raw = None
        passed = False
        try:
            request = urllib.request.Request(endpoint, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=60) as response:
                raw = json.load(response)
            passed = correct(expected, raw["answers"])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            error = f"{type(exc).__name__}: {exc}"
        report["results"].append({"group": group, "tag": tag, "expected": expected, "correct": passed,
                                  "elapsed_s": time.perf_counter() - started, "error": error,
                                  "payload": payload, "response": raw})
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    for group in sorted({r["group"] for r in report["results"]}):
        rows = [r for r in report["results"] if r["group"] == group]
        summary = {"group": group, "passed": sum(r["correct"] for r in rows), "total": len(rows)}
        report["summary"].append(summary)
        print(json.dumps(summary), flush=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Full diagnostic report: {output}\nThese checks are not route-following qualification.")
    return 2 if any(r["error"] for r in report["results"]) else 1 if any(not r["correct"] for r in report["results"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
