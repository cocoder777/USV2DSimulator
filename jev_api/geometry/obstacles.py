"""Nearby obstacle snapshots and simple swept clearance; never selects actions."""

import math
from geometry.coordinates import KNOT_MPS, xy


def nearby_obstacles(observation):
    boat = observation["boat"]
    origin = (boat["longitude"], boat["latitude"])
    hull = boat["collision_radius_m"]
    duration = observation["planning"]["action_duration_s"]
    execution = observation["execution"]
    maximum = max(abs(boat["targetSpeed"]), *observation["action_space"]["speeds_knots"]) * KNOT_MPS
    braking_distance = maximum * maximum / (2 * execution["max_deceleration_mps2"])
    turn_distance = maximum * execution["max_yaw_rate_deg_s"] / execution["max_angular_acceleration_deg_s2"]
    # Distances below are measured from the configured safety boundary, not its centre.
    search = max(braking_distance + maximum * 2 * duration, turn_distance + maximum * 2 * duration)
    selected = []
    for obstacle in observation.get("obstacles", []):
        center = xy((obstacle["longitude"], obstacle["latitude"]), origin)
        clearance = math.hypot(*center) - obstacle["avoidance_radius_m"] - hull
        if clearance <= search:
            selected.append({"id": obstacle["id"], "east_m": center[0], "north_m": center[1],
                             "radius_m": obstacle["radius_m"], "avoidance_radius_m": obstacle["avoidance_radius_m"],
                             "current_clearance_m": math.hypot(*center) - obstacle["radius_m"] - hull,
                             "current_avoidance_clearance_m": clearance,
                             "speed_mps": None if obstacle.get("targetSpeed") is None else obstacle["targetSpeed"] * KNOT_MPS,
                             "movement_course_deg_clockwise_from_north": obstacle.get("targetCourse")})
    return selected


def snapshot_clearance(path, obstacles, hull, radius_key="radius_m"):
    """Minimum hull-to-circle clearance over every path segment (including its start)."""
    minimum = math.inf
    for obstacle in obstacles:
        center = (obstacle["east_m"], obstacle["north_m"])
        for a, b in zip(path, path[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = dx * dx + dy * dy
            t = min(1.0, max(0.0, ((center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / length)) if length else 0
            distance = math.hypot(a[0] + t * dx - center[0], a[1] + t * dy - center[1])
            minimum = min(minimum, distance - obstacle[radius_key] - hull)
    return minimum


def clearance_facts(motion, future, obstacles, hull):
    clearance = snapshot_clearance(motion["path"], obstacles, hull)
    preview = snapshot_clearance(future["path"], obstacles, hull)
    end = snapshot_clearance([motion["endpoint"], motion["endpoint"]], obstacles, hull)
    future_end = snapshot_clearance([future["endpoint"], future["endpoint"]], obstacles, hull)
    safety_end = snapshot_clearance([future["endpoint"], future["endpoint"]], obstacles, hull, "avoidance_radius_m")
    safety = snapshot_clearance(motion["path"], obstacles, hull, "avoidance_radius_m")
    preview_safety = snapshot_clearance(future["path"], obstacles, hull, "avoidance_radius_m")
    return {"obstacle_hull_clearance_m": [clearance, end, preview, future_end],
            "obstacle_safety_clearance_m": [safety, preview_safety, safety_end]}



