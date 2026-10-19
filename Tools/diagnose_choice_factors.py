"""Local-only controlled tests of option count, labels and description length; no boat control."""
import argparse
import copy
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from boat_client import BACKENDS  # noqa: E402
from choice_control import ACTION_CHOICES  # noqa: E402

KEYS = list(ACTION_CHOICES)
TARGETS = (3, 24, 52)  # full reverse, coast, full forward; all with zero angular input


def subset(count):
    selected = set(TARGETS)
    for index in (round(i * 55 / (count - 1)) for i in range(count)):
        if len(selected) < count:
            selected.add(index)
    for index in range(56):
        if len(selected) < count:
            selected.add(index)
    return sorted(selected)


def cases():
    configurations = [("count-label", count, style, 0) for count in (3, 7, 14, 28, 56)
                      for style in ("neutral", "semantic", "numeric")]
    configurations += [("length", 14, "neutral", padding) for padding in (0, 4, 10)]
    linear = (-1.5, -1, -.5, 0, .375, .75, 1.125, 1.5)
    angular = (-60, -30, -15, 0, 15, 30, 60)
    for group, count, style, padding in configurations:
        indices = subset(count)
        for target in TARGETS:
            for position in (0, count // 2, count - 1):
                order = [index for index in indices if index != target]
                order.insert(position, target)
                criteria = {}
                for index in order:
                    key = f"k{index:02}" if style == "neutral" else KEYS[index]
                    description = " Available candidate." * padding
                    if style == "numeric":
                        action = ACTION_CHOICES[KEYS[index]]
                        description = f"a={linear[action['acceleration_index']]:g} m/s^2; alpha={angular[action['turn_index']]:g} deg/s^2."
                    criteria[key] = description.strip()
                expected = f"k{target:02}" if style == "neutral" else KEYS[target]
                yield {"group": group, "count": count, "style": style, "padding": padding,
                       "target_action": KEYS[target], "expected": expected, "expected_slot": position,
                       "payload": {"state": f"The explicitly required action label is {expected}.",
                                   "questions": {"action_0": {"type": "choice",
                                       "instructions": "Select exactly the option whose label matches the required action label in the state.",
                                       "criteria": criteria}}}}


def summary(rows):
    groups = {}
    for row in rows:
        key = (row["group"], row["count"], row["style"], row["padding"])
        groups.setdefault(key, []).append(row)
    return [{"group": key[0], "count": key[1], "style": key[2], "padding": key[3],
             "correct": sum(row["correct"] for row in group), "total": len(group),
             "errors": sum(row["error"] is not None for row in group),
             "input_tokens": sorted({row["raw_response"].get("usage", {}).get("input_tokens")
                                     for row in group if row["raw_response"]}),
             "chosen_slots": [row["chosen_slot"] for row in group],
             "expected_slots": [row["expected_slot"] for row in group]}
            for key, group in groups.items()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("laya", "kev"), default="laya")
    parser.add_argument("--suite", choices=("all", "counts", "length"), default="all")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new report path; existing reports must not be overwritten")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    endpoint, model, _ = BACKENDS[args.backend]
    report = {"backend": args.backend, "purpose": "Explicit label matching, not physical decision making or navigation",
              "endpoint": endpoint, "results": [], "summary": []}
    tests = [row for row in cases() if args.suite == "all" or
             (args.suite == "counts" and row["group"] == "count-label") or
             (args.suite == "length" and row["group"] == "length")]
    for number, case in enumerate(tests, 1):
        row = copy.deepcopy(case)
        payload = row["payload"]
        payload["model"] = model
        if args.backend == "laya":
            payload.update(max_len=4096, head_max_len=3072)
        started = time.perf_counter()
        raw, error, choice = None, None, None
        try:
            request = urllib.request.Request(endpoint, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=60) as response:
                raw = json.load(response)
            choice = raw["answers"]["action_0"]["choice"]
            if choice not in payload["questions"]["action_0"]["criteria"]:
                raise ValueError("Response choice does not exist in the supplied criteria")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            error = f"{type(exc).__name__}: {exc}"
        row.update(raw_response=raw, error=error, choice=choice, correct=choice == row["expected"],
                   chosen_slot=list(payload["questions"]["action_0"]["criteria"]).index(choice) if choice in payload["questions"]["action_0"]["criteria"] else None,
                   elapsed_s=time.perf_counter()-started)
        report["results"].append(row)
        if number % 9 == 0 or error:
            report["summary"] = summary(report["results"])
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        if number % 9 == 0:
            print(json.dumps(report["summary"][-1]), flush=True)
        if error and isinstance(error, str) and error.startswith("URLError"):
            print("Service unavailable; stopped without substituting any backend.", flush=True)
            break
    report["summary"] = summary(report["results"])
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved {len(report['results'])} requests to {args.output}", flush=True)
    return 2 if any(row["error"] for row in report["results"]) else 1 if any(not row["correct"] for row in report["results"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
