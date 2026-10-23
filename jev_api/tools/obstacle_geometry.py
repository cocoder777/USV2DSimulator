"""Nearby obstacle snapshots and simple swept clearance; never selects actions."""

import math
from tools.waypoint_geometry import KNOT_MPS, xy


AVOIDANCE_INSTRUCTIONS = (
    "HIGHEST PRIORITY: avoid collision with every nearby obstacle, before waypoint progress, "
    "speed ceilings, path directness or stopping at a requested location. Collision ends the mission. "
    "Obstacles MOVE; current positions, radii, movement speeds and courses are supplied when available. "
    "Clearance forecasts freeze obstacles at their current positions and are NOT safety guarantees. "
    "Maintain at least 1 m extra clearance when feasible. A preview below 1 m warns you to steer away EARLY, "
    "even if the direct action does not yet predict overlap. Do not creep straight toward a blocked path. "
    "Stopping does NOT stop moving obstacles: do not wait beside one; choose a detour that increases end clearance. "
    "A detour is allowed for avoidance; resume the current waypoint once clear. "
)


def nearby_obstacles(observation):
    boat = observation["boat"]
    origin = (boat["longitude"], boat["latitude"])
    hull = boat.get("collision_radius_m", 0.5)
    duration = observation["planning"]["action_duration_s"]
    transition = max(observation["execution"][key] for key in (
        "speed_transition_time_s", "course_transition_time_s"))
    maximum = max(abs(boat["targetSpeed"]), *observation["action_space"]["speeds_knots"]) * KNOT_MPS
    # No count cap: include every obstacle within the conservative local observation range.
    search = max(12.0, maximum * (2 * duration + transition) + 2.0)
    selected = []
    for obstacle in observation.get("obstacles", []):
        center = xy((obstacle["longitude"], obstacle["latitude"]), origin)
        clearance = math.hypot(*center) - obstacle["radius_m"] - hull
        if clearance <= search:
            selected.append({"id": obstacle["id"], "east_m": center[0], "north_m": center[1],
                             "radius_m": obstacle["radius_m"], "current_clearance_m": clearance,
                             "speed_mps": None if obstacle.get("targetSpeed") is None else obstacle["targetSpeed"] * KNOT_MPS,
                             "movement_course_deg_clockwise_from_north": obstacle.get("targetCourse")})
    return selected


def snapshot_clearance(path, obstacles, hull):
    """Minimum hull-to-circle clearance over every path segment (including its start)."""
    minimum = math.inf
    for obstacle in obstacles:
        center = (obstacle["east_m"], obstacle["north_m"])
        for a, b in zip(path, path[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = dx * dx + dy * dy
            t = min(1.0, max(0.0, ((center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / length)) if length else 0
            distance = math.hypot(a[0] + t * dx - center[0], a[1] + t * dy - center[1])
            minimum = min(minimum, distance - obstacle["radius_m"] - hull)
    return minimum


def describe_clearance(motion, future, obstacles, hull):
    clearance = snapshot_clearance(motion["path"], obstacles, hull)
    preview = snapshot_clearance(future["path"], obstacles, hull)
    end = snapshot_clearance([motion["endpoint"], motion["endpoint"]], obstacles, hull)
    future_end = snapshot_clearance([future["endpoint"], future["endpoint"]], obstacles, hull)
    return (
        ("OBSTACLE SAFETY MARGIN WARNING: below 1 m clearance; avoid this command if a safer detour is available. "
         if min(clearance, preview) < 1 else "")
        + f"Obstacle snapshot: minimum hull clearance {clearance:.3f} m during this action; "
        f"end clearance {end:.3f} m; same-command preview minimum {preview:.3f} m, end {future_end:.3f} m. "
        + ("COLLISION RISK: hull intersects an obstacle in this estimate. " if min(clearance, preview) <= 0 else "")
    )


def add_obstacle_state(state, observation, obstacles):
    state["obstacle_avoidance"] = {
        "boat_collision_radius_m": observation["boat"].get("collision_radius_m", 0.5),
        "nearby_obstacles": [
            {key: round(value, 3) if type(value) is float else value for key, value in obstacle.items()}
            for obstacle in obstacles
        ],
        "forecast": "Forecasts freeze current positions even when obstacle movement speed and course are supplied. Negative clearance means overlap.",
    }
