import math
import sys
import json
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import control_benchmark as benchmark


class GeometryTests(unittest.TestCase):
    def test_distance_to_polyline_uses_nearest_segment(self):
        polyline = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
        self.assertAlmostEqual(benchmark.distance_to_polyline((5.0, 2.0), polyline), 2.0)
        self.assertAlmostEqual(benchmark.distance_to_polyline((12.0, 5.0), polyline), 2.0)
        self.assertAlmostEqual(benchmark.distance_to_polyline((-2.0, 0.0), polyline), 2.0)

    def test_reference_polyline_prefers_curve_samples(self):
        route = {
            "waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 0}],
            "path": {"samples": [{"x": 0, "y": 0}, {"x": 5, "y": 2}, {"x": 10, "y": 0}]},
        }
        self.assertEqual(benchmark.route_polyline(route), [(0, 0), (5, 2), (10, 0)])

    def test_reference_polyline_falls_back_to_waypoints(self):
        route = {"waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 0}]}
        self.assertEqual(benchmark.route_polyline(route), [(0, 0), (10, 0)])


class ScenarioTests(unittest.TestCase):
    def test_rejects_scenario_without_waypoints(self):
        with self.assertRaises(ValueError):
            benchmark.validate_scenario({"name": "bad", "route": {"waypoints": []}})

    def test_accepts_minimal_scenario(self):
        scenario = {
            "name": "ahead",
            "route": {
                "waypoints": [
                    {"role": "start", "x": 0, "y": 0},
                    {"role": "end", "x": 10, "y": 0},
                ]
            },
            "max_simulation_seconds": 30,
        }
        benchmark.validate_scenario(scenario)


class MetricTests(unittest.TestCase):
    def test_summary_reports_completion_deviation_and_stop_error(self):
        scenario = {
            "name": "stop",
            "route": {
                "waypoints": [
                    {"role": "start", "x": 0, "y": 0},
                    {"role": "end", "x": 6, "y": 0},
                ],
                "arrival_radius_m": 1.0,
                "stop_speed_mps": 0.1,
            },
            "max_simulation_seconds": 30,
        }
        steps = [
            {"event": "step", "tick": 1, "simulation_time": 0.02, "x": 0.0, "y": 0.0,
             "heading_deg": 0.0, "speed": 0.1, "yaw_rate_deg": 0.0,
             "target_distance": 6.0, "bearing_deg": 0.0, "passed_count": 1, "completed": False},
            {"event": "step", "tick": 2, "simulation_time": 0.04, "x": 5.2, "y": 0.5,
             "heading_deg": 0.0, "speed": 0.05, "yaw_rate_deg": 0.0,
             "target_distance": 0.0, "bearing_deg": 0.0, "passed_count": 2, "completed": True},
        ]
        result = {"event": "scenario_end", "completed": True, "reason": "completed",
                  "passed_count": 2, "steps": 2, "simulation_time": 0.04}
        summary = benchmark.summarize_run(scenario, steps, result)
        self.assertTrue(summary["completed"])
        self.assertAlmostEqual(summary["final_distance_m"], math.hypot(.8, .5))
        self.assertAlmostEqual(summary["final_speed_mps"], 0.05)
        self.assertAlmostEqual(summary["stop_speed_excess_mps"], 0.0)
        self.assertAlmostEqual(summary["max_path_deviation_m"], 0.5)
        self.assertAlmostEqual(summary["mean_path_deviation_m"], 0.25)
        self.assertAlmostEqual(summary["mean_abs_heading_error_deg"], 0.0)
        self.assertEqual(summary["steps"], 2)

    def test_scenario_trace_accepts_string_scenario_label(self):
        trace = [
            {"event": "benchmark_start"},
            {"event": "scenario_start", "scenario": "ahead", "route": {"waypoints": []}},
            {"event": "step", "scenario": "ahead", "x": 0, "y": 0},
            {"event": "scenario_end", "scenario": "ahead", "completed": True, "reason": "completed"},
        ]
        records, result = benchmark.scenario_trace(trace, "ahead")
        self.assertEqual(len(records), 3)
        self.assertEqual(result["reason"], "completed")

    def test_summary_marks_timeout(self):
        scenario = {
            "name": "timeout",
            "route": {"waypoints": [{"role": "start", "x": 0, "y": 0}, {"role": "end", "x": 10, "y": 0}]},
            "max_simulation_seconds": 1,
        }
        steps = [{"event": "step", "tick": 1, "simulation_time": 1.0, "x": 0.0, "y": 0.0,
                  "heading_deg": 0.0, "speed": 0.0, "yaw_rate_deg": 0.0,
                  "target_distance": 10.0, "bearing_deg": 0.0, "passed_count": 1, "completed": False}]
        result = {"event": "scenario_end", "completed": False, "reason": "timeout",
                  "passed_count": 1, "steps": 1, "simulation_time": 1.0}
        summary = benchmark.summarize_run(scenario, steps, result)
        self.assertFalse(summary["completed"])
        self.assertEqual(summary["failure_reason"], "timeout")
        self.assertEqual(summary["outcome"], "navigation_failure")

    def test_technical_errors_are_not_navigation_failures(self):
        self.assertEqual(benchmark.classify_outcome(None), "incomplete")
        self.assertEqual(benchmark.classify_outcome({"reason": "wall_timeout"}), "technical_timeout")
        self.assertEqual(benchmark.classify_outcome({"reason": "runner_failed: WinError 10061"}), "service_interruption")
        self.assertEqual(benchmark.classify_outcome({"reason": "runner_failed: Decision service unavailable: timed out"}), "technical_timeout")
        self.assertEqual(benchmark.classify_outcome({"reason": "runner_failed: HTTP 500"}), "response_or_service_error")

    def test_unexecuted_scenario_has_no_fabricated_final_state(self):
        summary = benchmark.summarize_run({"name": "not_run"}, [], None)
        self.assertEqual(summary["outcome"], "incomplete")
        self.assertIsNone(summary["final_distance_m"])
        self.assertIsNone(summary["final_speed_mps"])
        self.assertIsNone(summary["mean_path_deviation_m"])

    def test_latency_matches_identity_instead_of_overlapping_ticks(self):
        scenario = {"name": "one", "route": {"waypoints": [{"x": 0, "y": 0}, {"x": 2, "y": 0}]}}
        trace = [{"event": "scenario_start", "scenario": "one"},
                 {"event": "request", "decision_id": "wanted"},
                 {"event": "step", "x": 1, "y": 0, "speed": .1},
                 {"event": "scenario_end", "completed": True, "reason": "completed"}]
        workers = [{"event": "decision", "request": {"decision_id": key}, "elapsed_s": latency,
                    "result": {"type": "decision_result", "action": {"acceleration_index": 3, "turn_index": 3}}}
                   for key, latency in (("wanted", .2), ("other_episode_same_tick", 99))]
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            (Path(directory) / "worker.jsonl").write_text("\n".join(json.dumps(row) for row in workers), encoding="utf-8")
            summary = benchmark.write_summary(directory, [scenario], trace)[0]
        self.assertEqual(summary["decision_latency_s"]["mean"], .2)
        self.assertEqual(summary["valid_responses"], 1)
        self.assertEqual(summary["action_counts"], {"3,3": 1})


if __name__ == "__main__":
    unittest.main()
