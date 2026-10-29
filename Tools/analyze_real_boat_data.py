"""Compare the supplied vessel log with the saved Unity scene (no runtime edits).

Run from any directory: python Tools/analyze_real_boat_data.py
Plotting requires matplotlib; the analysis itself uses only the standard library.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


ROOT = Path(__file__).resolve().parents[1]
KNOT = 1852 / 3600


def wrap(degrees):
    return (degrees + 180) % 360 - 180


def describe(values):
    ordered = sorted(values)
    return {
        "min": ordered[0],
        "p05": ordered[round((len(ordered) - 1) * .05)],
        "median": statistics.median(ordered),
        "p95": ordered[round((len(ordered) - 1) * .95)],
        "max": ordered[-1],
    }


def correlation(first, second):
    a = [x - statistics.mean(first) for x in first]
    b = [x - statistics.mean(second) for x in second]
    denominator = math.sqrt(sum(x * x for x in a) * sum(x * x for x in b))
    return sum(x * y for x, y in zip(a, b)) / denominator if denominator else None


def crossing(times, speeds, threshold):
    if threshold <= speeds[0]:
        return times[0]
    for i in range(1, len(speeds)):
        if speeds[i - 1] < threshold <= speeds[i]:
            fraction = (threshold - speeds[i - 1]) / (speeds[i] - speeds[i - 1])
            return times[i - 1] + fraction * (times[i] - times[i - 1])
    return None


def analyze(source, scene):
    # Positional indexing preserves the two different columns both named 时间.
    with source.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = list(reader)
    if len(header) != 37 or len(rows) < 41 or any(len(row) != 37 for row in rows):
        raise ValueError("Expected the original 37-column vessel log, with >=41 rows")
    expected = {0: "时间", 9: "目标绝对方向", 10: "目标绝对速度", 12: "目标纬度",
                13: "目标经度", 14: "时间戳", 22: "时间", 23: "东向速度",
                25: "航向角", 26: "航向角加速度", 28: "北向速度"}
    if any(header[index] != name for index, name in expected.items()):
        raise ValueError("Unexpected column order")
    col = lambda index: [float(row[index]) for row in rows]
    speed, east, north, latitude, longitude, heading, course = (
        col(index) for index in (10, 23, 28, 12, 13, 25, 9)
    )
    numeric = [value for index in (10, 23, 28, 12, 13, 25, 9) for value in col(index)]
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("Non-finite navigation measurements")
    times = []
    for row in rows:
        minute, second = map(float, row[22].split(":"))
        value = minute * 60 + second
        if times and value < times[-1] - 1800:
            value += 3600 * math.ceil((times[-1] - value) / 3600)
        times.append(value)
    times = [value - times[0] for value in times]
    intervals = [b - a for a, b in zip(times, times[1:])]
    if any(abs(value - .1) > 1e-7 for value in intervals):
        raise ValueError("Window estimates require uninterrupted 0.1-second sampling")
    unwrapped = [heading[0]]
    for previous, current in zip(heading, heading[1:]):
        unwrapped.append(unwrapped[-1] + wrap(current - previous))

    # Local WGS84 displacement; mean-latitude meridional/prime-vertical radii.
    dx, dy, ve, vn = [], [], [], []
    for i, dt in enumerate(intervals, 1):
        phi = math.radians((latitude[i] + latitude[i - 1]) / 2)
        q = 1 - 6.69437999014e-3 * math.sin(phi) ** 2
        rn = 6378137 / math.sqrt(q)
        rm = 6378137 * (1 - 6.69437999014e-3) / q ** 1.5
        dx.append(math.radians(longitude[i] - longitude[i - 1]) * rn * math.cos(phi))
        dy.append(math.radians(latitude[i] - latitude[i - 1]) * rm)
        ve.append((east[i - 1] + east[i]) / 2 * dt)
        vn.append((north[i - 1] + north[i]) / 2 * dt)
    units = {}
    for name, scale in (("mps", 1), ("knots", KNOT), ("kmph", 1 / 3.6)):
        units[name] = {
            "integrated_path_m": sum(math.hypot(x, y) * scale for x, y in zip(ve, vn)),
            "velocity_vector_rmse_mps": math.sqrt(statistics.mean(
                ((x - e * scale) / dt) ** 2 + ((y - n * scale) / dt) ** 2
                for x, y, e, n, dt in zip(dx, dy, ve, vn, intervals))),
        }

    text = scene.read_text(encoding="utf-8-sig")
    components = [block for block in text.split("--- !u!")
                  if "m_EditorClassIdentifier: Assembly-CSharp::BoatDynamics2D" in block]
    if len(components) != 1:
        raise ValueError("Expected one serialized BoatDynamics2D component")
    fields = ("maxForwardSpeed", "maxReverseSpeed", "maxAcceleration", "maxDeceleration",
              "maxAngularAccelerationDeg", "maxYawRateDeg", "lengthMetres", "beamMetres")
    limits = {field: float(re.search(r"^  " + field + r": (\S+)", components[0], re.M)[1])
              for field in fields}
    windows, series = {}, {}
    for width in (1, 5, 10, 20):
        span = width * .1
        acceleration = [(speed[i + width] - speed[i]) * KNOT / span
                        for i in range(len(rows) - width)]
        yaw = [(unwrapped[i + width] - unwrapped[i]) / span
               for i in range(len(rows) - width)]
        # Difference of consecutive interval-mean yaw rates, centered at i+width.
        alpha = [(unwrapped[i + 2 * width] - 2 * unwrapped[i + width] + unwrapped[i]) / span ** 2
                 for i in range(len(rows) - 2 * width)]
        peak = max(range(len(alpha)), key=lambda i: abs(alpha[i]))
        windows[str(span)] = {
            "acceleration_mps2": describe(acceleration), "yaw_deg_s": describe(yaw),
            "angular_acceleration_deg_s2": describe(alpha),
            "acceleration_above_unity_count": sum(x > limits["maxAcceleration"] for x in acceleration),
            "angular_acceleration_above_unity_count": sum(abs(x) > limits["maxAngularAccelerationDeg"] for x in alpha),
            "angular_peak_center_s": times[peak + width],
        }
        series[span] = ([(times[i] + times[i + width]) / 2 for i in range(len(acceleration))],
                        acceleration, yaw, times[width:-width], alpha)
    bands = []
    for lo, hi in ((speed[0], 10), (10, 20), (20, 30), (30, 40), (40, 45),
                   (45, 48), (48, 49), (49, 50)):
        start, stop = crossing(times, speed, lo), crossing(times, speed, hi)
        bands.append({"from_knots": lo, "to_knots": hi, "start_s": start, "stop_s": stop,
                      "mean_acceleration_mps2": (hi - lo) * KNOT / (stop - start)
                      if start is not None and stop is not None else None})
    delays = []
    for row in rows:
        _, minute, second = map(float, row[0].split(":"))
        inner_minute, inner_second = map(float, row[22].split(":"))
        delays.append((minute * 60 + second - inner_minute * 60 - inner_second + 1800) % 3600 - 1800)
    gyro_correlations = {}
    for index in (18, 19, 20):
        values = col(index)
        averaged = [statistics.mean(values[i:i + 11]) for i in range(len(rows) - 10)]
        gyro_correlations[header[index]] = correlation(averaged, series[1.0][2])
    summary = {
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "rows": len(rows), "duration_s": times[-1], "sample_interval_s": describe(intervals),
        "inner_time_range": [rows[0][22], rows[-1][22]],
        "outer_minus_inner_s": describe(delays), "timestamp_unique_values": len(set(col(14))),
        "speed_knots": describe(speed),
        "speed_norm_max_error": max(abs(v - math.hypot(e, n)) for v, e, n in zip(speed, east, north)),
        "course_max_error_deg": max(abs(wrap(c - math.degrees(math.atan2(e, n))))
                                    for c, e, n in zip(course, east, north)),
        "position_path_m": sum(math.hypot(x, y) for x, y in zip(dx, dy)),
        "units_hypotheses": units,
        "fitted_mps_per_logged_speed": sum(x * e + y * n for x, y, e, n in zip(dx, dy, ve, vn))
                                      / sum(e * e + n * n for e, n in zip(ve, vn)),
        "heading_minus_course_deg_speed_at_least_10_knots": describe(
            [wrap(h - c) for h, c, v in zip(heading, course, speed) if v >= 10]),
        "heading_wrap_count": sum(abs(b - a) > 180 for a, b in zip(heading, heading[1:])),
        "heading_angular_acceleration_column_all_zero": all(x == 0 for x in col(26)),
        "sensor_column_ranges_units_unconfirmed": {header[index]: describe(col(index))
                                                   for index in range(15, 21)},
        "gyro_vs_heading_rate_1s_correlation": gyro_correlations,
        "all_empty_columns": [header[i] for i in range(len(header)) if all(row[i] == "" for row in rows)],
        "all_zero_status_columns": [header[i] for i in (21, 24, 31, 32, 33, 34, 35, 36)
                                    if all(x == 0 for x in col(i))],
        "unity_scene_limits": limits, "windows_s": windows, "speed_bands": bands,
        "time_to_50_knots_observed_s": crossing(times, speed, 50),
        "time_to_50_knots_unity_s": (50 - speed[0]) * KNOT / limits["maxAcceleration"],
        "assumptions": ["Navigation speed units inferred from position, not a device specification",
                        "Times are relative to the first inner time; date and hour not inferred",
                        "Window estimates are observations, not identified maximum capabilities",
                        "Constant full-acceleration Unity comparison is conditional on a held 50-knot target",
                        "No throttle/rudder commands, IMU mounting, gravity correction, or sensor units supplied"],
    }
    return summary, (times, speed, unwrapped, series)


def plot(destination, summary, data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times, speed, heading, series = data
    limits = summary["unity_scene_limits"]
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    axis = axes[0, 0]
    axis.plot(times, speed, label="Measured speed (unit inferred: knots)")
    axis.plot(times, [min(limits["maxForwardSpeed"], speed[0] * KNOT + limits["maxAcceleration"] * t) / KNOT
                     for t in times], label="Unity held 50 kn target, full acceleration", linestyle="--")
    axis.set(ylabel="Speed (kn)", title="Speed response: measured run vs model assumption")
    axis = axes[0, 1]
    for span in (.1, 1.0):
        centers, acceleration, _, _, _ = series[span]
        axis.plot(centers, acceleration, label=f"{span:g} s finite difference", alpha=.8)
    axis.axhline(limits["maxAcceleration"], color="red", linestyle="--", label="Unity acceleration limit")
    axis.set(ylabel="Tangential acceleration (m/s^2)", title="Speed derivative (not an IMU axis)")
    axis = axes[1, 0]
    for span in (.1, 1.0):
        centers, _, yaw, _, _ = series[span]
        axis.plot(centers, yaw, label=f"{span:g} s heading derivative", alpha=.8)
    for sign in (-1, 1):
        axis.axhline(sign * limits["maxYawRateDeg"], color="red", linestyle="--",
                     label="Unity yaw limits" if sign == 1 else None)
    axis.set(ylabel="Yaw rate (deg/s, clockwise positive)", title="Observed turns do not establish maximum capability")
    axis = axes[1, 1]
    for span in (.5, 1.0, 2.0):
        _, _, _, centers, alpha = series[span]
        axis.plot(centers, alpha, label=f"{span:g} s interval width", alpha=.8)
    for sign in (-1, 1):
        axis.axhline(sign * limits["maxAngularAccelerationDeg"], color="red", linestyle="--",
                     label="Unity angular acceleration limits" if sign == 1 else None)
    axis.set(ylabel="Heading second difference (deg/s^2)", title="Angular estimates depend strongly on window width")
    for axis in axes.flat:
        axis.set_xlabel("Time since first sample (s)")
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
    figure.suptitle("0-50KN.csv: 35.6 s / 357 samples; observed response is not a capability envelope")
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "docs/甲方接口/0-50KN.csv")
    parser.add_argument("--scene", type=Path, default=ROOT / "Assets/Scenes/Main.unity")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "Logs/real_boat_analysis")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    summary, data = analyze(args.csv, args.scene)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    destination = args.output_dir / "summary.json"
    destination.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(destination)
    if not args.no_plot:
        destination = args.output_dir / "comparison.png"
        plot(destination, summary, data)
        print(destination)


if __name__ == "__main__":
    main()
