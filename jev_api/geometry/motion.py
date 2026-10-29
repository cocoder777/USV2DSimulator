"""Bounded speed and yaw forecasts. Computes consequences; never chooses an action."""

import math


def bounded_turn(delta, initial_yaw, maximum_yaw, acceleration, t):
    """Same accelerate/coast/brake angular trajectory as Unity BoatMotion.Turn."""
    if t <= 0:
        return 0.0, initial_yaw
    if abs(delta) < 1e-7 and abs(initial_yaw) < 1e-7:
        return delta, 0.0
    switching = delta - initial_yaw * abs(initial_yaw) / (2 * acceleration)
    direction = (1 if initial_yaw == 0 else -math.copysign(1, initial_yaw)) if switching == 0 else math.copysign(1, switching)
    velocity, distance = direction * initial_yaw, direction * delta
    peak = math.sqrt(max(0.0, acceleration * distance + velocity * velocity / 2))
    top = min(maximum_yaw, peak)
    accelerating = max(0.0, (top - velocity) / acceleration)
    coasting = max(0.0, (distance - (2 * top * top - velocity * velocity) / (2 * acceleration)) / top) if peak > maximum_yaw else 0.0
    braking = top / acceleration
    if t >= accelerating + coasting + braking:
        return delta, 0.0
    span = min(t, accelerating)
    angle = velocity * span + acceleration * span * span / 2
    velocity += acceleration * span
    t -= span
    span = min(t, coasting)
    angle += velocity * span
    t -= span
    span = min(t, braking)
    angle += velocity * span - acceleration * span * span / 2
    velocity -= acceleration * span
    return direction * angle, direction * velocity


def predict_ground_motion(speed_mps, target_speed_mps, target_course_deg, execution, duration, dt=None, *, route_details=True):
    """Integrate at the plant timestep, including measured angular state at rest."""
    dt = execution["fixed_dt_s"] if dt is None else dt
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(dt) or dt <= 0:
        raise ValueError("Prediction duration and timestep must be positive")
    acceleration, deceleration = execution["max_acceleration_mps2"], execution["max_deceleration_mps2"]
    yaw_limit, angular = execution["max_yaw_rate_deg_s"], execution["max_angular_acceleration_deg_s2"]
    # Unity reports CCW yaw internally; the extension uses standard clockwise course/yaw.
    start_course = execution["heading_course_deg"]
    initial_angle = 90 - start_course
    initial_yaw = -execution["yaw_rate_deg_s"]
    target_angle = initial_angle if target_speed_mps == 0 else 90 - target_course_deg
    delta = (target_angle - initial_angle) % 360
    delta = delta - 360 if delta > 180 else delta
    active = execution.get("active_target_course_deg")
    if (target_speed_mps > 0 and active is not None
            and abs((target_course_deg - active + 180) % 360 - 180) < 0.0001):
        delta = -execution["active_course_error_deg"]
    rate = acceleration if target_speed_mps >= speed_mps else deceleration

    def speed_at(t):
        difference = target_speed_mps - speed_mps
        return speed_mps + math.copysign(min(abs(difference), rate * t), difference)

    def velocity(t):
        angle = math.radians(initial_angle + bounded_turn(delta, initial_yaw, yaw_limit, angular, t)[0])
        speed = speed_at(t)
        return speed * math.cos(angle), speed * math.sin(angle)

    def integrate(end, capture=False):
        x = y = 0.0
        path = [[0.0, 0.0]] if capture else []
        speeds = [speed_at(0)] if capture and route_details else []
        count = math.ceil(end / dt)
        for i in range(count):
            start, stop = i * dt, min((i + 1) * dt, end)
            step = stop - start
            samples = [velocity(start + step * k / 4) for k in range(5)]
            weights = (1, 4, 2, 4, 1)
            x += sum(w * p[0] for w, p in zip(weights, samples)) * step / 12
            y += sum(w * p[1] for w, p in zip(weights, samples)) * step / 12
            if capture:
                path.append([x, y])
                if route_details:
                    speeds.append(speed_at(stop))
        return [x, y], path, speeds

    midpoint = integrate(duration / 2)[0] if route_details else None
    endpoint, path, speeds = integrate(duration, True)
    angle, yaw = bounded_turn(delta, initial_yaw, yaw_limit, angular, duration)
    result = {"endpoint": endpoint, "end_speed": speed_at(duration),
            "end_course_deg": (90 - initial_angle - angle) % 360, "end_yaw_rate_deg_s": -yaw,
            "path": path}
    return {"midpoint": midpoint, **result, "path_speeds": speeds} if route_details else result


def braking_facts(speed_mps, execution, ceiling_mps=0, action_duration=1):
    deceleration = execution["max_deceleration_mps2"]
    yaw = abs(execution["yaw_rate_deg_s"])
    return {
        "time_to_stop_s": abs(speed_mps) / deceleration,
        "distance_to_stop_m": speed_mps**2 / (2 * deceleration),
        "distance_to_ceiling_m": max(0, speed_mps**2 - ceiling_mps**2) / (2 * deceleration),
        "one_decision_travel_m": abs(speed_mps) * action_duration,
        "angular_braking_angle_deg": yaw**2 / (2 * execution["max_angular_acceleration_deg_s2"]),
        "time_to_cancel_yaw_s": yaw / execution["max_angular_acceleration_deg_s2"],
    }


def preview_duration(execution, duration, speed_mps):
    """A fixed two-action view cannot show a 12 deg/s turn or full-speed braking."""
    turn_time = 180 / execution["max_yaw_rate_deg_s"] + 2 * execution["max_yaw_rate_deg_s"] / execution["max_angular_acceleration_deg_s2"]
    return max(2 * duration, turn_time, abs(speed_mps) / execution["max_deceleration_mps2"] + duration)
