"""Approximate ground-velocity forecasts; never chooses or ranks actions."""

import math


def _course_offset(delta, transition_time, t):
    """Smooth a course change assuming zero initial course rotation."""
    if transition_time <= 0 or t >= transition_time:
        return delta
    u = t / transition_time
    return (-2 * u**3 + 3 * u**2) * delta


def predict_ground_motion(
    speed_mps,
    course_deg,
    target_speed_mps,
    target_course_deg,
    speed_time,
    course_time,
    duration,
    dt=0.02,
):
    """Estimate motion in fixed east/north axes from ground velocity only.

    Bow, yaw and actuator progress are unavailable. At rest, use the candidate
    direction solely for this estimate; actual startup/turn motion can differ.
    """
    initial_speed = abs(speed_mps)
    start = target_course_deg if course_deg is None else course_deg
    initial_angle = 90 - start
    difference = ((90 - target_course_deg) - initial_angle) % 360
    delta = (
        (difference - 360 if difference > 180 else difference)
        if target_speed_mps > 0
        else 0
    )
    c, s = math.cos(math.radians(initial_angle)), math.sin(math.radians(initial_angle))

    def speed_at(t):
        return (
            target_speed_mps
            if speed_time <= 0 or t >= speed_time
            else initial_speed + (target_speed_mps - initial_speed) * t / speed_time
        )

    def velocity(t):
        heading = math.radians(_course_offset(delta, course_time, t))
        speed = speed_at(t)
        return speed * math.cos(heading), speed * math.sin(heading)

    def boundaries(end):
        return sorted(
            set(
                [0.0, end]
                + [i * dt for i in range(1, math.ceil(end / dt)) if i * dt < end]
                + [t for t in (speed_time, course_time) if 0 < t < end]
            )
        )

    def rotate(point):
        return [point[0] * c - point[1] * s, point[0] * s + point[1] * c]

    path = [[0.0, 0.0]]

    def integrate(times, capture=False):
        x = y = 0.0
        for t, stop in zip(times, times[1:]):
            step = stop - t
            samples = [velocity(t + step * k / 4) for k in range(5)]
            weights = (1, 4, 2, 4, 1)
            x += sum(w * p[0] for w, p in zip(weights, samples)) * step / 12
            y += sum(w * p[1] for w, p in zip(weights, samples)) * step / 12
            if capture:
                path.append([x, y])
        # Preserve the established local-axis rounding before rotation so the
        # refactor does not change boundary classifications or model wording.
        return rotate([0.0 if abs(v) < 0.0005 else v for v in (x, y)])

    times = boundaries(duration)
    midpoint = integrate(boundaries(duration / 2))
    endpoint = integrate(times, capture=True)
    return {
        "midpoint": midpoint,
        "endpoint": endpoint,
        "end_speed": speed_at(duration),
        "path": [rotate(p) for p in path],
        "path_speeds": [speed_at(t) for t in times],
    }
