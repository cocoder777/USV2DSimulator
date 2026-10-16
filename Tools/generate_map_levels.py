"""Build reproducible level 1..4 tasks; leave the original level 0 files untouched."""
import argparse
import copy
import json
import math
from pathlib import Path

from generate_fixed_curve import build_curve, serialized

DATA_DIRECTORY = Path(__file__).resolve().parent.parent / "Assets" / "StreamingAssets"
# Uneven X spacing and alternating Y offsets add progressively more steering.
# Levels 1/2 retain quintic y(x); levels 3/4 use ordered parametric 2D curves.
LAYOUTS = {
    1: [(0, 0), (10, 2), (21, -2), (31, 4), (43, -1), (54, 3), (66, 0)],
    2: [(0, 0), (10, 3), (22, -3), (33, 5), (46, -4), (58, 2), (69, -5), (82, 4), (94, 0)],
    3: [(0, 0), (18, 0), (30, 10), (18, 20), (0, 22), (-12, 34),
        (2, 46), (22, 44), (34, 56), (18, 68), (0, 66)],
    # Same nine checkpoint IDs, compact spacing, deliberately scrambled visit order.
    4: [(0, 0), (6.4, 0), (-3.2, 6.4), (8, 8), (-4.8, -1.6), (3.2, 9.6),
        (9.6, -3.2), (-4, 2.4), (4.8, 4), (0, -4.8), (7.2, 5.6)],
}


def build_return_curve(task, step=0.1):
    """C1 cubic Hermite curve, sampled by travelled distance; retain exact checkpoint samples."""
    if not math.isfinite(step) or step <= 0:
        raise ValueError("Sampling step must be finite and positive")
    points = [(p["x"], p["y"]) for p in task["waypoints"]]
    tangents = [(points[1][0] - points[0][0], points[1][1] - points[0][1])]
    tangents += [tuple((points[i+1][axis] - points[i-1][axis]) / 2 for axis in (0, 1))
                 for i in range(1, len(points)-1)]
    tangents += [(points[-1][0] - points[-2][0], points[-1][1] - points[-2][1])]
    samples = [{"x": points[0][0], "y": points[0][1]}]
    indices = [0]
    for segment, (a, b) in enumerate(zip(points, points[1:])):
        m, n = tangents[segment:segment+2]
        # Dense interpolation followed by distance resampling (not X-axis sampling).
        count = max(100, math.ceil((math.dist(a, b) + math.hypot(*m) + math.hypot(*n)) / (step / 10)))
        previous = a
        remaining = step
        for j in range(1, count+1):
            t = j / count
            h0, h1 = 2*t**3 - 3*t*t + 1, -2*t**3 + 3*t*t
            h2, h3 = t**3 - 2*t*t + t, t**3 - t*t
            current = tuple(h0*a[k] + h1*b[k] + h2*m[k] + h3*n[k] for k in (0, 1))
            length = math.dist(previous, current)
            while length >= remaining:
                position = tuple(previous[k] + (current[k]-previous[k]) * remaining/length for k in (0, 1))
                samples.append({"x": round(position[0], 8), "y": round(position[1], 8)})
                previous = position
                length = math.dist(previous, current)
                remaining = step
            remaining -= length
            previous = current
        if math.dist((samples[-1]["x"], samples[-1]["y"]), b) < 1e-6:
            samples[-1] = {"x": b[0], "y": b[1]}
        else:
            samples.append({"x": b[0], "y": b[1]})
        indices.append(len(samples)-1)
    task["schema_version"] = 3
    task["path"] = {"type": "sampled_curve", "interpolation": "parametric_cubic_hermite",
                    "sampling_step_m": step, "waypoint_sample_indices": indices, "samples": samples}
    return task


def build_level(base, level):
    if level not in LAYOUTS:
        raise ValueError("Generated map level must be 1..4")
    task = copy.deepcopy(base)
    layout = LAYOUTS[level]
    task["waypoints"] = [
        {"id": "S" if i == 0 else "E" if i == len(layout) - 1 else f"P{i}",
         "role": "start" if i == 0 else "end" if i == len(layout) - 1 else "checkpoint",
         "x": float(x), "y": float(y)}
        for i, (x, y) in enumerate(layout)
    ]
    task = build_return_curve(task) if level >= 3 else build_curve(task)
    task["route_id"] = f"usv_map_level_{level}"
    return task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    base = json.loads((DATA_DIRECTORY / "fixed_route.json").read_text(encoding="utf-8"))
    outputs = [(DATA_DIRECTORY / "Maps" / f"level_{level}.json", build_level(base, level))
               for level in LAYOUTS]
    for path, task in outputs:
        if args.check:
            if not path.exists() or json.loads(path.read_text(encoding="utf-8")) != task:
                parser.exit(1, f"Map is missing or stale: {path}\n")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(serialized(task), encoding="utf-8")
    print("Verified maps 1..4" if args.check else "Generated maps 1..4 in StreamingAssets")


if __name__ == "__main__":
    main()
