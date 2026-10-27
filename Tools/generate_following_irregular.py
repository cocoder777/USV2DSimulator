"""Generate a repeatable, Level-4-inspired target schedule, private to Unity.

The scrambled directions come from the route map generator; this is not a
waypoint tracking task. Jev receives only the target's current observation.
Unity's FollowingScenarioData.Parse performs the authoritative swept validation.
"""
import argparse
import json
import math
from pathlib import Path
import random

from generate_map_levels import LAYOUTS

OUTPUT = Path(__file__).resolve().parent.parent / "Assets/StreamingAssets/Following/irregular.json"
SEED = 20261027


def build():
    rng = random.Random(SEED)
    segments = []
    elapsed, course = 0.0, 0.0

    def add(duration, speed, yaw=0):
        nonlocal elapsed, course
        if elapsed >= 232:
            return
        duration = min(duration, 232 - elapsed)
        segments.append(dict(start_s=round(elapsed, 4), speed_knots=speed,
                             yaw_rate_deg_s=round(yaw, 7)))
        elapsed = round(elapsed + duration, 4)
        course = (course + yaw * duration) % 360

    points = LAYOUTS[4]
    headings = [(math.degrees(math.atan2(b[0]-a[0], b[1]-a[1])) - 90) % 360
                for a, b in zip(points, points[1:])]
    index = 0
    while elapsed < 232:
        desired = (headings[index % len(headings)] + (rng.uniform(-12, 12) if index else 0)) % 360
        delta = (desired - course + 180) % 360 - 180
        if abs(delta) > .1:
            duration = math.ceil(abs(delta) / rng.choice([8, 10, 12]) * 10) / 10
            add(duration, rng.choice([8, 10, 12, 14]), delta / duration)
        add(rng.randint(4, 10), rng.choice([6, 10, 12, 16]))
        if index % 3 == 2:
            add(rng.randint(4, 6), 0)
        index += 1
    segments.append(dict(start_s=232, speed_knots=0, yaw_rate_deg_s=0))
    data = dict(scenario_id=f"following_irregular_seed_{SEED}", duration_s=240,
                initial_state=dict(x=0, y=0, heading_deg=90, speed=10*1852/3600, yaw_rate_deg=0),
                target=dict(x=0, y=150, course_deg=0, speed_knots=10, radius_m=6,
                            acceleration_mps2=1.46, deceleration_mps2=3.5),
                segments=segments, obstacles=[])
    # Approximate path only for deterministic layout construction. Runtime and
    # final validation both use Unity's exact integrator, not these samples.
    positions = []
    x, y, speed, course, index = 0., 150., 10*1852/3600, 0., 0
    for tick in range(12001):
        t = tick * .02
        while index+1 < len(segments) and segments[index+1]["start_s"] <= t+1e-7:
            index += 1
        segment = segments[index]
        desired = segment["speed_knots"]*1852/3600
        change = (1.46 if desired >= speed else 3.5)*.02
        next_speed = speed + max(-change, min(change, desired-speed))
        angle = math.radians(course+segment["yaw_rate_deg_s"]*.01)
        positions.append((x, y))
        x += math.sin(angle)*(speed+next_speed)*.01
        y += math.cos(angle)*(speed+next_speed)*.01
        course = (course+segment["yaw_rate_deg_s"]*.02) % 360
        speed = next_speed
    sampled = positions[::10]
    # Three nearby hazards, each safely outside the complete target patrol.
    # The full oscillation disk is reserved, not only its initial phase.
    for radius, safety, amplitude, frequency, fraction in [
            (4, 50, 5, .06, .15), (10, 60, 0, 0, .5), (4, 45, 8, .05, .85)]:
        anchor = sampled[int(fraction*(len(sampled)-1))]
        candidates = []
        bound = safety + amplitude + 6
        for cx in range(math.floor(min(p[0] for p in sampled))-100,
                        math.ceil(max(p[0] for p in sampled))+101, 10):
            for cy in range(math.floor(min(p[1] for p in sampled))-100,
                            math.ceil(max(p[1] for p in sampled))+101, 10):
                if math.hypot(cx, cy) < safety+amplitude+6.1+15:
                    continue
                if any(math.hypot(cx-o["x"], cy-o["y"]) < safety+amplitude+o["avoidance_radius_m"]+o["amplitude_m"]+15
                       for o in data["obstacles"]):
                    continue
                clearance = min(math.hypot(cx-px, cy-py) for px, py in sampled)-bound
                if 12 <= clearance <= 22:
                    candidates.append((math.hypot(cx-anchor[0], cy-anchor[1]), cx, cy))
        if not candidates:
            raise ValueError("No safe nearby obstacle placement")
        _, cx, cy = min(candidates)
        data["obstacles"].append(dict(x=cx, y=cy, radius_m=radius, avoidance_radius_m=safety,
                                       amplitude_m=amplitude, frequency_rad_s=frequency, axis_course_deg=90))
    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    data = build()
    if args.check:
        if json.loads(OUTPUT.read_text(encoding="utf-8")) != data:
            parser.exit(1, "Irregular scenario is stale\n")
    else:
        OUTPUT.write_text(json.dumps(data, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(f"{'Verified' if args.check else 'Generated'} {len(data['segments'])} segments, 240s, seed {SEED}")
