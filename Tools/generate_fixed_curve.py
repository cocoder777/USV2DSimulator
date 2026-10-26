"""Generate the Level 0 reference curve directly in Unity StreamingAssets."""
import argparse
import copy
import json
import math
from pathlib import Path

STEP_X_METERS = 2.0
INTERPOLATION = "piecewise_quintic_smoothstep"


def evaluate_segment(start, end, t):
    """Return the curve Y coordinate for 0 <= t <= 1."""
    height = end["y"] - start["y"]
    return start["y"] + height * (10 * t**3 - 15 * t**4 + 6 * t**5)


def build_curve(route, step=STEP_X_METERS):
    if not math.isfinite(step) or step <= 0:
        raise ValueError("X sampling step must be finite and positive")
    points = route["waypoints"]
    if len(points) < 2:
        raise ValueError("At least two task points are required")
    ids = set()
    for index, point in enumerate(points):
        expected = "start" if index == 0 else "end" if index == len(points) - 1 else "checkpoint"
        if not isinstance(point.get("id"), str) or not point["id"].strip() or point["id"] in ids:
            raise ValueError("Task point IDs must be non-empty and unique")
        ids.add(point["id"])
        if point.get("role") != expected:
            raise ValueError("Expected start, checkpoints and end in order")
        if any(type(point.get(axis)) not in (int, float) or not math.isfinite(point[axis]) for axis in ("x", "y")):
            raise ValueError("Task point coordinates must be finite numbers")
        if index and point["x"] <= points[index - 1]["x"]:
            raise ValueError("Quintic y(x) requires strictly increasing task point X")
    samples = [{"x": points[0]["x"], "y": points[0]["y"]}]
    for start, end in zip(points, points[1:]):
        width = end["x"] - start["x"]
        count = math.ceil(round(width / step, 10))
        for index in range(1, count + 1):
            if index == count:
                samples.append({"x": end["x"], "y": end["y"]})
            else:
                x = start["x"] + index * step
                y = evaluate_segment(start, end, (x - start["x"]) / width)
                samples.append({"x": round(x, 10), "y": round(y, 10)})
    result = copy.deepcopy(route)
    result["schema_version"] = 2
    result["route_id"] = "usv_fixed_curve_001"
    result["path"] = {
        "type": "sampled_curve",
        "interpolation": INTERPOLATION,
        "sampling_step_x_m": step,
        "samples": samples,
    }
    return result


def serialized(value):
    return json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify the bundled route without writing")
    args = parser.parse_args()
    route_file = Path(__file__).resolve().parent.parent / "Assets" / "StreamingAssets" / "fixed_route.json"
    try:
        current = json.loads(route_file.read_text(encoding="utf-8"))
        profile = json.loads((Path(__file__).parent / "map_profile.json").read_text(encoding="utf-8"))
        layout = [(0, 0), (10, 0), (20, 5), (30, 5), (40, 0)]
        scale = profile["coordinate_scale"]
        speed_limits = profile["waypoint_speed_knots"]
        for i, (point, (x, y)) in enumerate(zip(current["waypoints"], layout)):
            point.update(x=x * scale, y=y * scale,
                routeSpeed=(0 if i == 0 else profile["final_speed_knots"] if i == len(layout) - 1
                            else speed_limits[(i - 1) % len(speed_limits)]) * 1852 / 3600)
        current["arrival_radius_m"] = math.hypot(11.6, 3.4) / 2 * profile["waypoint_radius_multiplier"]
        current["stop_speed_mps"] = profile["stop_speed_mps"]
        route = build_curve(current, step=profile["sampling_step_m"])
        if args.check:
            if json.loads(route_file.read_text(encoding="utf-8")) != route:
                parser.exit(1, "Bundled route is stale; regenerate it.\n")
        else:
            route_file.write_text(serialized(route), encoding="utf-8")
        print(f"{'Verified' if args.check else 'Generated'} {len(route['path']['samples'])} curve samples; task points preserved.")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"Curve generation failed: {error}\n")


if __name__ == "__main__":
    main()
