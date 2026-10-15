import copy
import json
import math
from pathlib import Path
import unittest

from generate_fixed_curve import build_curve, evaluate_segment, serialized, sync_interface

DIRECTORY = Path(__file__).resolve().parent


class FixedCurveTests(unittest.TestCase):
    def setUp(self):
        self.route = json.loads((DIRECTORY / "fixed_route.json").read_text(encoding="utf-8"))
        self.info = json.loads((DIRECTORY / "algorithm_interface.json").read_text(encoding="utf-8"))

    def test_shared_data_and_repeatable_generation(self):
        generated = build_curve(self.route)
        self.assertEqual(generated, self.route)
        self.assertEqual(generated, self.info["route"])
        self.assertEqual(sync_interface(self.info, generated), self.info)
        self.assertEqual(serialized(build_curve(generated)), serialized(generated))

    def test_order_and_all_task_points(self):
        samples = self.route["path"]["samples"]
        self.assertEqual(len(samples), 401)
        self.assertEqual(self.route["waypoints"], [
            {"id": "S", "role": "start", "x": 0.0, "y": 0.0},
            {"id": "P1", "role": "checkpoint", "x": 10.0, "y": 0.0},
            {"id": "P2", "role": "checkpoint", "x": 20.0, "y": 5.0},
            {"id": "P3", "role": "checkpoint", "x": 30.0, "y": 5.0},
            {"id": "E", "role": "end", "x": 40.0, "y": 0.0}])
        for index, sample in enumerate(samples):
            self.assertTrue(all(math.isfinite(value) for value in sample.values()))
            self.assertAlmostEqual(sample["x"], index * 0.1)
        for index, point in enumerate(self.route["waypoints"]):
            self.assertEqual(samples[index * 100], {"x": point["x"], "y": point["y"]})
        self.assertEqual(self.route["initial_state"], {"x": 0, "y": 0, "heading_deg": 0, "speed": 0, "yaw_rate_deg": 0})
        self.assertEqual((self.route["cruise_speed_mps"], self.route["arrival_radius_m"], self.route["stop_speed_mps"]), (1.5, 1.0, 0.1))

    def test_continuous_tangents_and_curvature(self):
        points = self.route["waypoints"]
        for start, end in zip(points, points[1:]):
            left, right = evaluate_segment(start, end, 0), evaluate_segment(start, end, 1)
            self.assertAlmostEqual(left[0], start["y"])
            self.assertAlmostEqual(right[0], end["y"])
            self.assertEqual(left[1:3], (0, 0))
            self.assertEqual(right[1:3], (0, 0))
        for index in range(1, len(points) - 1):
            left = evaluate_segment(points[index - 1], points[index], 1)
            right = evaluate_segment(points[index], points[index + 1], 0)
            self.assertEqual(left[:3], right[:3])

    def test_samples_follow_formula_and_retain_straights(self):
        points = self.route["waypoints"]
        for index, sample in enumerate(self.route["path"]["samples"]):
            segment = min(index // 100, 3)
            start, end = points[segment], points[segment + 1]
            t = (sample["x"] - start["x"]) / (end["x"] - start["x"])
            self.assertAlmostEqual(sample["y"], evaluate_segment(start, end, t)[0], places=9)
            self.assertGreaterEqual(sample["y"], min(start["y"], end["y"]))
            self.assertLessEqual(sample["y"], max(start["y"], end["y"]))

    def test_ideal_angular_requirements_within_current_limits(self):
        speed = self.route["cruise_speed_mps"]
        parameters = self.info["model_parameters"]
        peak_rate = peak_alpha = peak_command = 0
        points = self.route["waypoints"]
        for start, end in zip(points, points[1:]):
            for index in range(2001):
                _, p, q, r = evaluate_segment(start, end, index / 2000)
                curvature = q / (1 + p*p)**1.5
                d_curvature_ds = r / (1 + p*p)**2 - 3*p*q*q / (1 + p*p)**3
                rate = math.degrees(speed * curvature)
                alpha = math.degrees(speed**2 * d_curvature_ds)
                command = alpha + parameters["angularDamping"]["value"] * rate
                peak_rate, peak_alpha, peak_command = max(peak_rate, abs(rate)), max(peak_alpha, abs(alpha)), max(peak_command, abs(command))
        self.assertAlmostEqual(peak_rate, 21.19, places=2)
        self.assertLess(peak_rate, parameters["maxYawRateDeg"]["value"])
        self.assertLess(peak_command, parameters["maxAngularAccelerationDeg"]["value"])
        print(f"Ideal constant-speed curve: yaw rate <= {peak_rate:.2f} deg/s; net yaw acceleration <= {peak_alpha:.2f} deg/s^2; damping-compensated command <= {peak_command:.2f} deg/s^2 (actuator lag excluded).")

    def test_invalid_generation_inputs(self):
        for step in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                build_curve(self.route, step)
        for mutate in (
            lambda r: r["waypoints"][1].update(x=0),
            lambda r: r["waypoints"][1].update(y=float("nan")),
            lambda r: r["waypoints"][1].update(id="S"),
            lambda r: r["waypoints"][0].update(role="end"),
        ):
            route = copy.deepcopy(self.route)
            mutate(route)
            with self.assertRaises(ValueError):
                build_curve(route)


if __name__ == "__main__":
    unittest.main()
