"""Shared wording helpers for model requests."""

from geometry.obstacles import clearance_facts


def position_text(point):
    east, north = point
    return (
        f"{abs(east):.3f} m "
        + ("east" if east >= 0 else "west")
        + " and "
        + f"{abs(north):.3f} m "
        + ("north" if north >= 0 else "south")
    )


AVOIDANCE_INSTRUCTIONS = (
    "HIGHEST PRIORITY: avoid collision with every nearby obstacle, before waypoint progress, "
    "speed ceilings, path directness or stopping at a requested location. Collision ends the mission. "
    "Obstacles MOVE; current positions, radii, movement speeds and courses are supplied when available. "
    "Clearance forecasts freeze obstacles at their current positions and are NOT safety guarantees. "
    "Keep the own hull outside each configured avoidance safety circle. Negative safety clearance warns you to steer away EARLY, "
    "even if the direct action does not yet predict overlap. Do not creep straight toward a blocked path. "
    "Stopping does NOT stop moving obstacles: do not wait beside one; choose a detour that increases end clearance. "
    "A detour is allowed for avoidance; resume the current waypoint once clear. "
    "If already inside a safety circle, every action's minimum includes that initial violation. Compare the LONG preview's END safety clearance to exit it; "
    "do not repeatedly stop at its edge. With bounded turning, a low-speed turn may briefly move closer before escaping; preserve physical collision clearance. "
)

def describe_clearance(motion, future, obstacles, hull):
    facts = clearance_facts(motion, future, obstacles, hull)
    clearance, end, preview, future_end = facts["obstacle_hull_clearance_m"]
    safety, preview_safety, safety_end = facts["obstacle_safety_clearance_m"]
    return (
        ("AVOIDANCE SAFETY CIRCLE WARNING: predicted safety-boundary intrusion; choose an early bypass. "
         if min(safety, preview_safety) < 0 else "")
        + f"Obstacle snapshot: minimum physical hull clearance {clearance:.3f} m during this action; "
        f"end clearance {end:.3f} m; same-command preview minimum {preview:.3f} m, end {future_end:.3f} m. "
        + f"Safety-circle clearance after subtracting own hull: action {safety:.3f} m; preview minimum {preview_safety:.3f} m, end {safety_end:.3f} m. "
        + ("COLLISION RISK: hull intersects an obstacle in this estimate. " if min(clearance, preview) <= 0 else "")
    )

def add_obstacle_state(state, observation, obstacles):
    state["obstacle_avoidance"] = {
        "boat_collision_radius_m": observation["boat"]["collision_radius_m"],
        "nearby_obstacles": [
            {key: round(value, 3) if type(value) is float else value for key, value in obstacle.items()}
            for obstacle in obstacles
        ],
        "forecast": "Forecasts freeze current positions even when obstacle movement speed and course are supplied. Negative physical clearance means collision; negative safety-circle clearance means a safety-distance violation, not physical contact.",
    }
