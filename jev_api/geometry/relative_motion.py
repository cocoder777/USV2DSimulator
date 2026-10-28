"""Relative motion and swept target-range facts; no request construction."""

import math
from geometry.coordinates import KNOT_MPS

def direction(course):
    angle = math.radians(course)
    return math.sin(angle), math.cos(angle)

def velocity(speed_knots, course):
    if course is None:
        return 0.0, 0.0
    return tuple(speed_knots * KNOT_MPS * v for v in direction(course))

def relative_facts(delta, own_velocity, target_velocity, forward):
    distance = math.hypot(*delta)
    relative = [t - b for t, b in zip(target_velocity, own_velocity)]
    return {"distance_m": distance,
            "range_rate_mps": sum(a * b for a, b in zip(delta, relative)) / distance if distance > 1e-9 else 0.0,
            "along_behind_m": sum(a * b for a, b in zip(delta, forward)),
            "cross_track_m": -delta[0] * forward[1] + delta[1] * forward[0]}

def line_reference_facts(delta, reference):
    """Reference on the current boat-target line, measured from own position."""
    distance = math.hypot(*delta)
    if distance <= 1e-9:
        return {"defined": False, "east_m": 0.0, "north_m": 0.0,
                "range_error_m": reference, "line_of_sight_course_deg": None, "away_course_deg": None}
    unit = [c / distance for c in delta]
    bearing = math.degrees(math.atan2(unit[0], unit[1])) % 360
    return {"defined": True, "east_m": delta[0] - reference * unit[0],
            "north_m": delta[1] - reference * unit[1], "range_error_m": abs(distance - reference),
            "line_of_sight_course_deg": bearing, "away_course_deg": (bearing + 180) % 360}

def course_error(bearing, course):
    """Signed shortest clockwise error; opposite headings consistently use -180."""
    return (bearing - course + 180) % 360 - 180

def moving_range(motion, target_position, target_velocity, duration):
    """Relative straight sweeps at the forecast samples; includes time zero."""
    path = motion["path"]
    # predict_ground_motion uses uniform dt and may have a shorter final step.
    dt = motion["sample_dt_s"]
    relative = [[target_position[j] + target_velocity[j] * min(i * dt, duration) - p[j]
                 for j in (0, 1)] for i, p in enumerate(path)]
    minimum, maximum = math.inf, 0.0
    for a, b in zip(relative, relative[1:]):
        v = [b[j] - a[j] for j in (0, 1)]
        vv = sum(c * c for c in v)
        t = min(1.0, max(0.0, -sum(a[j] * v[j] for j in (0, 1)) / vv)) if vv else 0
        minimum = min(minimum, math.hypot(*(a[j] + t * v[j] for j in (0, 1))))
        maximum = max(maximum, math.hypot(*a), math.hypot(*b))
    return minimum, maximum, relative[-1]
