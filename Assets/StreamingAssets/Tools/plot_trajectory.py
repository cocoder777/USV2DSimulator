"""Render a completed or failed Unity run and obstacle traces as a standalone PNG."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def plot_run(document, output):
    route = document["route"]
    points = route["waypoints"]
    reference = (route.get("path") or {}).get("samples") or points
    actual = document["samples"]
    if not reference or not actual:
        raise ValueError("Reference and actual trajectory must contain samples")
    fig, ax = plt.subplots(figsize=(12, 5.2), layout="constrained")
    try:
        ax.plot([p["x"] for p in reference], [p["y"] for p in reference],
                color="black", linestyle="--", linewidth=1.6, label="Reference", zorder=3)
        modes = list(dict.fromkeys(p["mode"] for p in (actual[1:] or actual)))
        ax.plot([p["x"] for p in actual], [p["y"] for p in actual],
                color="#0072BD", linewidth=1.5, label="Actual (" + "/".join(modes) + ")", zorder=4)
        radius = route["arrival_radius_m"]
        for p in points:
            ax.add_patch(Circle((p["x"], p["y"]), radius, fill=False,
                                edgecolor="0.6", linewidth=0.9, zorder=2))
            ax.annotate(p["id"], (p["x"], p["y"] + radius), xytext=(0, 4),
                        textcoords="offset points", ha="center", va="bottom", fontsize=9)
            ax.plot(p["x"], p["y"], ".", color="0.5", markersize=3, zorder=2)
        ax.plot(actual[0]["x"], actual[0]["y"], "o", color="#0072BD", markersize=4, zorder=5)
        ax.plot(actual[-1]["x"], actual[-1]["y"], "s", color="#0072BD", markersize=4, zorder=5)
        obstacles = {}
        for sample in actual:
            for obstacle in sample.get("obstacles") or []:
                obstacles.setdefault(obstacle["id"], []).append(obstacle)
        for index, trail in enumerate(obstacles.values()):
            ax.plot([p["x"] for p in trail], [p["y"] for p in trail], color="red",
                    alpha=0.4, linewidth=0.8)
            last = trail[-1]
            ax.add_patch(Circle((last["x"], last["y"]), last["radius_m"], fill=False,
                                edgecolor="red", linewidth=1.2,
                                label="Moving obstacles (final positions)" if index == 0 else None))
            safety = last.get("avoidance_radius_m")
            if safety is not None:
                ax.add_patch(Circle((last["x"], last["y"]), safety, fill=False,
                                    edgecolor="darkorange", linestyle=":", linewidth=1,
                                    label="Avoidance safety circles" if index == 0 else None))
        last_sample = actual[-1]
        ax.add_patch(Circle((last_sample["x"], last_sample["y"]), document.get("boat_radius_m", radius),
                            fill=False, edgecolor="#0072BD", linewidth=1, label="Boat hull (final)"))
        if document["result"] == "failed":
            ax.plot(actual[-1]["x"], actual[-1]["y"], "x", color="red", markersize=9, zorder=6)
        ax.set_aspect("equal", adjustable="datalim")
        ax.margins(x=0.06, y=0.25)
        ax.set_xlabel("World X (m)")
        ax.set_ylabel("World Y (m)")
        level = document.get("map_level")
        name = f"Level {level}" if level is not None else route["route_id"]
        ax.set_title(f"{name}: reference and actual trajectory")
        ax.grid(True, color="0.9", linewidth=0.6)
        ax.legend(loc="upper left")
        fig.text(0.99, 0.01,
                 f"{document['result']} | simulation {document['simulation_time']:.2f} s"
                 f" | final speed {document['final_speed_mps']:.3f} m/s",
                 ha="right", va="bottom", fontsize=8, color="0.35")
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=180, facecolor="white")
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    document = json.loads(args.data.read_text(encoding="utf-8-sig"))
    plot_run(document, args.output or args.data.with_suffix(".png"))


if __name__ == "__main__":
    main()
