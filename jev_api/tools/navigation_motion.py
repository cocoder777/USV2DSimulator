"""Motion of a held speed/course target; describes options without selecting one."""

import math


def delta_angle(angle):
    result = angle % 360
    return result - 360 if result > 180 else result


def turn_response(delta, initial_yaw, remaining, t):
    if remaining <= 0 or t >= remaining:
        return delta, 0.0
    u = t / remaining
    angle = (-2 * u**3 + 3 * u**2) * delta + (
        u**3 - 2 * u**2 + u
    ) * remaining * initial_yaw
    yaw = (
        (-6 * u**2 + 6 * u) * delta + (3 * u**2 - 4 * u + 1) * remaining * initial_yaw
    ) / remaining
    return angle, yaw


def predict_motion(
    initial_speed,
    target_speed,
    speed_time,
    delta,
    initial_yaw,
    turn_time,
    duration,
    dt,
    include_path=False,
):
    path = [[0.0, 0.0]]

    def speed_at(t):
        return (
            target_speed
            if speed_time <= 0 or t >= speed_time
            else initial_speed + (target_speed - initial_speed) * t / speed_time
        )

    def velocity(t):
        heading = math.radians(turn_response(delta, initial_yaw, turn_time, t)[0])
        speed = speed_at(t)
        return speed * math.cos(heading), speed * math.sin(heading)

    def integrate(end, capture=False):
        x = y = 0.0
        boundaries = sorted(
            set(
                [0.0, end]
                + [i * dt for i in range(1, math.ceil(end / dt)) if i * dt < end]
                + [t for t in (speed_time, turn_time) if 0 < t < end]
            )
        )
        for t, stop in zip(boundaries, boundaries[1:]):
            step = stop - t
            samples = [velocity(t + step * k / 4) for k in range(5)]
            weights = (1, 4, 2, 4, 1)
            x += sum(w * p[0] for w, p in zip(weights, samples)) * step / 12
            y += sum(w * p[1] for w, p in zip(weights, samples)) * step / 12
            if capture:
                path.append([x, y])
        return [0.0 if abs(v) < 0.0005 else v for v in (x, y)]

    angle, yaw = turn_response(delta, initial_yaw, turn_time, duration)
    result = {
        "midpoint": integrate(duration / 2),
        "endpoint": integrate(duration, include_path),
        "end_speed": speed_at(duration),
        "end_heading": angle,
        "end_yaw": yaw,
    }
    if include_path:
        result["path"] = path
    return result
