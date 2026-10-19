"""Recompute comparable metrics without altering any original run artifacts."""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import control_benchmark as benchmark
from classic_control import predict_action

sys.path.insert(0, str(benchmark.ROOT / "jev_api"))
from choice_control import decode_action


def audit_execution(records, workers):
    """Reuse the existing predictor to check complete holds against real Unity states."""
    states = {record["tick"]: record for record in records if record.get("event") == "step"}
    checked, maximum_error, raw_mismatches = 0, 0.0, 0
    for record in records:
        if record.get("event") != "request":
            continue
        worker = workers.get(record["decision_id"], {})
        result = worker.get("result", {})
        action = result.get("action")
        if not action:
            continue
        raw = worker.get("raw_response")
        if raw is not None:
            raw_mismatches += decode_action(raw) != action
        observation = record["observation"]
        end_tick = record["tick"] + round(observation["planning"]["action_duration_s"] / .02)
        actual = states.get(end_tick)
        if actual is None:
            continue  # Final hold can end early on mission completion/simulation budget.
        predicted = predict_action(observation, action["acceleration_index"], action["turn_index"])
        error = max(abs(predicted.x - actual["x"]), abs(predicted.y - actual["y"]),
                    abs(predicted.speed_mps - actual["speed"]), abs(predicted.yaw_rate_deg_s - actual["yaw_rate_deg"]),
                    abs((predicted.heading_deg - actual["heading_deg"] + 180) % 360 - 180))
        maximum_error = max(maximum_error, error)
        checked += 1
    return {"complete_holds_checked": checked, "max_numeric_error_m_mps_deg_deg_s": maximum_error,
            "raw_action_mismatches": raw_mismatches, "passed": checked > 0 and maximum_error < .01 and raw_mismatches == 0}


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def write_csv(path, rows):
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def compare(runs, output):
    output.mkdir(parents=True, exist_ok=False)
    aggregate, flat, decisions, source_hashes = {}, [], [], []
    expected_scenarios = None
    expected_observations = {}
    for label, directory in runs:
        scenarios = benchmark.load_scenarios(directory / "scenarios.json")
        if expected_scenarios is None:
            expected_scenarios = scenarios
        if scenarios != expected_scenarios:
            raise ValueError("Scenario configuration differs: " + label)
        for filename in ("scenarios.json", "unity-trace.jsonl", "worker.jsonl"):
            source_hashes.append({"controller": label, "path": str(directory / filename),
                                  "sha256": hashlib.sha256((directory / filename).read_bytes()).hexdigest()})
        trace = benchmark.read_jsonl(directory / "unity-trace.jsonl")
        # Write recalculated summaries only in a new derived directory.
        derived = output / label
        derived.mkdir()
        (derived / "worker.jsonl").write_bytes((directory / "worker.jsonl").read_bytes())
        summaries = benchmark.write_summary(derived, scenarios, trace, label)
        (derived / "worker.jsonl").unlink()  # Original worker log remains the authority.
        workers = benchmark.read_jsonl(directory / "worker.jsonl")
        worker_by_id = {row.get("request", {}).get("decision_id"): row for row in workers if row.get("event") == "decision"}
        for scenario, summary in zip(scenarios, summaries):
            records, end = benchmark.scenario_trace(trace, scenario["name"])
            requests = [r for r in records if r.get("event") == "request"]
            initial = scenario["route"]["initial_state"]
            first = requests[0] if requests else None
            summary["initial_state_verified"] = bool(first and first["tick"] == 0 and
                all(abs(first[k] - initial[k]) < 1e-4 for k in ("x", "y", "heading_deg", "speed", "yaw_rate_deg")))
            observation = first.get("observation") if first else None
            if scenario["name"] not in expected_observations:
                expected_observations[scenario["name"]] = observation
            summary["initial_observation_matches_baseline"] = bool(observation and observation == expected_observations[scenario["name"]])
            if summary["outcome"] == "navigation_failure" and (not end or end["simulation_time"] + .03 < scenario["max_simulation_seconds"]):
                summary["outcome"] = "incomplete"
            if not summary["initial_state_verified"]:
                summary["outcome"] = "invalid_initial_state"
            elif not summary["initial_observation_matches_baseline"]:
                summary["outcome"] = "invalid_test_conditions"
            summary["full_budget_s"] = scenario["max_simulation_seconds"]
            summary["execution_audit"] = audit_execution(records, worker_by_id)
            if summary["execution_audit"]["complete_holds_checked"] and not summary["execution_audit"]["passed"]:
                summary["outcome"] = "invalid_execution_chain"
            flat.append({"controller": label, **{k: v for k, v in summary.items() if not isinstance(v, dict)}})
            polyline = benchmark.route_polyline(scenario["route"])
            # Exactly one row per model/controller decision, preserving the raw output in worker.jsonl.
            for record in requests:
                worker = worker_by_id.get(record["decision_id"], {})
                result = worker.get("result", {})
                action = result.get("action", {})
                decisions.append({"controller": label, "scenario": scenario["name"],
                    "decision_id": record["decision_id"], "tick": record["tick"],
                    **{k: record.get(k) for k in ("x", "y", "speed", "heading_deg", "yaw_rate_deg", "target_distance", "bearing_deg")},
                    "path_deviation_m": benchmark.distance_to_polyline((record["x"], record["y"]), polyline),
                    "elapsed_s": worker.get("elapsed_s"), "response_type": result.get("type"),
                    **action, "error": result.get("error")})
        counts = {}
        for row in summaries:
            counts[row["outcome"]] = counts.get(row["outcome"], 0) + 1
        valid = counts.get("success", 0) + counts.get("navigation_failure", 0)
        aggregate[label] = {"source": str(directory), "outcomes": counts, "evaluated_navigation_trials": valid,
            "navigation_success_rate": counts.get("success", 0) / valid if valid else None,
            "planned_trials": len(scenarios), "scenarios": summaries}
        save_json(derived / "summary.json", aggregate[label])
    save_json(output / "comparison.json", aggregate)
    save_json(output / "source-hashes.json", source_hashes)
    write_csv(output / "scenarios.csv", flat)
    write_csv(output / "decisions.csv", decisions)
    return aggregate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="append", required=True, help="label=run_directory")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare([(value.split("=", 1)[0], Path(value.split("=", 1)[1])) for value in args.run], args.output)
    print(json.dumps({name: {k: v for k, v in row.items() if k != "scenarios"} for name, row in result.items()}, indent=2))


if __name__ == "__main__":
    main()
