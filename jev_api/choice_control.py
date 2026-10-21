"""Jev chooses ground-speed/course commands using waypoint-only global observations."""

import math
from tools.waypoint_geometry import WaypointGeometry, KNOT_MPS
from tools.navigation_motion import predict_ground_motion

DIRECTIONS = (
    "north",
    "northeast",
    "east",
    "southeast",
    "south",
    "southwest",
    "west",
    "northwest",
)
ACTION_CHOICES = {
    f"speed_{i}__{name}": (i, j) for i in range(8) for j, name in enumerate(DIRECTIONS)
}


def position_text(point):
    east, north = point
    return (
        f"{abs(east):.3f} m "
        + ("east" if east >= 0 else "west")
        + " and "
        + f"{abs(north):.3f} m "
        + ("north" if north >= 0 else "south")
    )


BACKGROUND = (
    "Navigate a boat through mandatory waypoint circles IN ORDER, then stop inside the final circle. "
    "There is no prescribed path between points: choose your own efficient path. "
    "All positions and directions use FIXED geographic east/north axes, NEVER the bow or camera. "
    "No obstacles, wind or sideways drift are simulated. The world pauses while you decide. "
    "Each waypoint has a MAXIMUM permitted actual speed while passing its circle; this is only a ceiling, "
    "not a desired speed or a speed to match. Slower is allowed. "
    "You choose one nonnegative target ground speed and one ABSOLUTE course, clockwise from north. "
    "The simulator actuator turns the boat smoothly; a course command is not an instantaneous movement direction."
)
INSTRUCTIONS = (
    "Choose ONE command. Treat a predicted passing_speed_above_ceiling=true as a violation to avoid. "
    "Speed ceilings are mandatory and take priority over faster progress. An EXCEEDS warning must not be traded for speed. "
    "If excess is already unavoidable from current motion, brake strongly to reduce it rather than accelerate. "
    "Never accelerate hard near a circle merely because this single action stops short of entering it. "
    "Use the longer same-command preview to anticipate the following action. "
    "On final approach use a slow controlled entry under its ceiling, then stop. "
    "Do not stop far outside and then restart at full speed: approach at a low nonzero speed instead. "
    "First pass the current mandatory circle without exceeding its speed ceiling. "
    "Future points are previews only; never skip the current point. "
    "Brake BEFORE entering a slower waypoint circle: the ceiling applies to ACTUAL speed, not commanded speed. "
    "Because forecasts are approximate, begin braking with spatial margin: do not rely on barely meeting the ceiling at the exact boundary. "
    "Use approach_margin_speed_above_ceiling as an early-braking warning, especially with residual high speed or a course change. "
    "The extra approach margin is a conservative cue only; the actual arrival radius and speed ceiling are unchanged. "
    "Account for speed and course transition durations. A zero-speed command brakes over time, not instantly. "
    "Intermediate waypoints require no stop. Once a waypoint is passed, proceed to the next one; "
    "keep respecting the previous ceiling until you leave its circle. "
    "If recently_passed_circle exists, its exit ceiling is still ACTIVE even though the target switched. "
    "A candidate with previous_circle_speed_above_ceiling=true accelerates too early: delay acceleration until outside. "
    "The final waypoint requires being inside AND slow enough; entering quickly is not completion. "
    "A stopped boat outside the final circle must approach instead of repeatedly stopping. "
    "Avoid needless stops, crawling, detours or alternating large course changes. "
    "After a turn settles, increase speed again when feasible; do not keep the slowest startup speed for a long clear approach. "
    "Among commands that respect active speed ceilings and reach the correct circle, compare distance_reduction_m: "
    "prefer useful larger progress over repeatedly crawling for a negligible endpoint improvement. "
    "Travel fast when far enough away, aligned, and able to brake before the next circle; slow for turns or nearby targets. "
    "For an off-axis target, diagonal motion or alternating adjacent compass headings can reach it. "
    "Inspect the predicted endpoint distance and entry facts, but these are APPROXIMATIONS from ground velocity: "
    "bow direction, yaw rate and actuator progress are unavailable. In particular startup and sharp turns may differ. "
    "If the waypoint is behind the current movement, slow before turning back. "
    "When very close or already inside the final circle, prefer braking to zero over another accelerating move. "
    "All candidates remain available: no local controller selects, ranks or filters your choice."
)


def _candidate_facts(
    observation,
    geometry,
    target_speed,
    target_course,
    direction,
    distance,
    limit,
    previous,
):
    task = observation["task"]
    boat = observation["boat"]
    execution = observation["execution"]
    speed = abs(boat["targetSpeed"]) * KNOT_MPS
    course = boat["targetCourse"]
    duration = observation["planning"]["action_duration_s"]
    motion = predict_ground_motion(
        speed,
        course,
        target_speed,
        target_course,
        execution["speed_transition_time_s"],
        execution["course_transition_time_s"],
        duration,
    )
    facts = geometry.motion_facts(motion)
    margin_facts = geometry.motion_facts(motion, radius=task["arrival_radius_m"] * 1.25)
    future = predict_ground_motion(
        speed,
        course,
        target_speed,
        target_course,
        execution["speed_transition_time_s"],
        execution["course_transition_time_s"],
        duration * 2,
    )
    future_facts = geometry.motion_facts(future)
    candidate = {
        "command": {
            "speed_mps": round(target_speed, 6),
            "absolute_course_deg": target_course,
            "direction": direction,
        },
        "approximate_effect": {
            "midpoint": position_text(motion["midpoint"]),
            "endpoint": position_text(motion["endpoint"]),
            "end_speed_mps": round(motion["end_speed"], 6),
            "distance_reduction_m": round(
                distance - facts["end_checkpoint_distance_m"], 3
            ),
            **{
                k: (round(v, 6) if type(v) is float else v)
                for k, v in facts.items()
                if k in ("max_speed_in_circle_mps", "end_checkpoint_distance_m")
            },
            "passing_speed_above_ceiling": facts["max_speed_in_circle_mps"] is not None
            and facts["max_speed_in_circle_mps"] > limit + 1e-5,
            "approach_margin_speed_above_ceiling": margin_facts[
                "max_speed_in_circle_mps"
            ]
            is not None
            and margin_facts["max_speed_in_circle_mps"] > limit + 1e-5,
            "same_command_preview": {
                "duration_s": duration * 2,
                "end_checkpoint_distance_m": round(
                    future_facts["end_checkpoint_distance_m"], 3
                ),
                "enters_checkpoint": future_facts["enters_checkpoint"],
                "passing_speed_above_ceiling": future_facts["max_speed_in_circle_mps"]
                is not None
                and future_facts["max_speed_in_circle_mps"] > limit + 1e-5,
            },
        },
    }
    if previous is not None:
        previous_facts = geometry.motion_facts(motion, previous)
        previous_peak = previous_facts["max_speed_in_circle_mps"]
        previous_limit = task["planWaypointList"][previous]["routeSpeed"]
        candidate["approximate_effect"].update(
            {
                "previous_circle_max_speed_mps": (
                    None if previous_peak is None else round(previous_peak, 6)
                ),
                "previous_circle_speed_above_ceiling": previous_peak is not None
                and previous_peak > previous_limit + 1e-5,
            }
        )
    return candidate


def _describe_candidate(command, effect, duration, distance, limit, previous_limit):
    preview = effect["same_command_preview"]
    text = (
        f"Set target ground speed {command['speed_mps']:.3f} m/s, absolute course {command['absolute_course_deg']:.0f} degrees ({command['direction']}). "
        f"Approximate motion over {duration:.3f} s: midpoint {effect['midpoint']}; endpoint {effect['endpoint']}; end speed {effect['end_speed_mps']:.3f} m/s. "
        f"Target distance {distance:.3f} -> {effect['end_checkpoint_distance_m']:.3f} m; distance reduction {effect['distance_reduction_m']:.3f} m. "
    )
    peak = effect["max_speed_in_circle_mps"]
    if peak is None:
        text += "Current waypoint circle not entered during this estimate. "
    else:
        text += (
            f"Inside current waypoint circle: estimated maximum speed {peak:.3f} m/s, permitted ceiling {limit:.3f} m/s; "
            + (
                "EXCEEDS the ceiling. "
                if effect["passing_speed_above_ceiling"]
                else "does not exceed the ceiling. "
            )
        )
    if effect["approach_margin_speed_above_ceiling"]:
        text += "Early braking warning: still above the ceiling in the extra approach margin. "
    text += (
        f"If held for {preview['duration_s']:.3f} s instead (hypothetical only): end distance {preview['end_checkpoint_distance_m']:.3f} m; "
        + (
            "would enter the circle and EXCEEDS its speed ceiling. "
            if preview["passing_speed_above_ceiling"]
            else (
                "would enter the circle without exceeding its ceiling. "
                if preview["enters_checkpoint"]
                else "circle not entered yet. "
            )
        )
    )
    if "previous_circle_max_speed_mps" in effect:
        previous_peak = effect["previous_circle_max_speed_mps"]
        text += (
            f"Before exiting the PREVIOUS circle: estimated maximum speed {previous_peak:.3f} m/s, permitted ceiling {previous_limit:.3f} m/s; "
            + (
                "EXCEEDS the previous ceiling. "
                if effect["previous_circle_speed_above_ceiling"]
                else "does not exceed the previous ceiling. "
            )
        )
    if command["speed_mps"] == 0:
        text += "This is BRAKING TO STOP, not instant stopping; its course value is ignored by the simulator. "
    text += "Choose when this movement respects active speed ceilings, reaches the required circle and makes efficient progress."
    return text


def build_payload(observation, model="jev-latest"):
    geometry = WaypointGeometry(observation)
    task, boat = observation["task"], observation["boat"]
    execution = observation["execution"]
    duration = observation["planning"]["action_duration_s"]
    speed = abs(boat["targetSpeed"]) * KNOT_MPS
    course = boat["targetCourse"]
    target = geometry.target
    distance = math.hypot(*target)
    final = task["next_waypoint_index"] == task["planWaypointNum"] - 1
    limit = task["planWaypointList"][task["next_waypoint_index"]]["routeSpeed"]
    state = {
        "task": {
            "background": BACKGROUND,
            "stage": (
                (
                    "final_parking"
                    if distance <= task["arrival_radius_m"]
                    else "final_approach"
                )
                if final
                else "checkpoint"
            ),
            "current_point_after_start": task["next_waypoint_index"],
            "total_points_after_start": task["planWaypointNum"] - 1,
            "final_stop_threshold_mps": task["stop_speed_mps"],
        },
        "boat": {
            "actual_speed_mps": round(speed, 6),
            "movement_course_deg_clockwise_from_north": course,
            "course_available": course is not None,
        },
        "target": {
            "position": position_text(target),
            "distance_m": round(distance, 3),
            "arrival_radius_m": task["arrival_radius_m"],
            "inside_arrival_circle": distance <= task["arrival_radius_m"],
            "passing_speed_ceiling_mps": limit,
            "forecast_approach_margin_m": task["arrival_radius_m"] * 0.25,
        },
        "upcoming_waypoints": [
            {
                "index": p["index"],
                "position": position_text(p["east_north_m"]),
                "passing_speed_ceiling_mps": p["speed_limit_mps"],
            }
            for p in geometry.upcoming[1:]
        ],
        "execution": {
            "duration_s": duration,
            **execution,
            "frame": "Fixed east/north; course 0=N,90=E,180=S,270=W. No bow-relative positions.",
            "forecast": "Approximate ground-velocity forecast; assumes zero initial course rotation and a fresh response. Unknown startup direction uses candidate direction. Not a guarantee.",
        },
    }
    # A passed circle remains subject to its ceiling until the boat exits it.
    previous = task["next_waypoint_index"] - 1
    if (
        previous > 0
        and math.hypot(*geometry.waypoints[previous]) <= task["arrival_radius_m"]
    ):
        state["recently_passed_circle"] = {
            "position": position_text(geometry.waypoints[previous]),
            "radius_m": task["arrival_radius_m"],
            "speed_ceiling_mps": task["planWaypointList"][previous]["routeSpeed"],
        }
    active_previous = previous if "recently_passed_circle" in state else None
    previous_limit = (
        task["planWaypointList"][previous]["routeSpeed"]
        if active_previous is not None
        else None
    )
    criteria = {}
    for name, (i, j) in ACTION_CHOICES.items():
        candidate = _candidate_facts(
            observation,
            geometry,
            observation["action_space"]["speeds_knots"][i] * KNOT_MPS,
            observation["action_space"]["courses_deg"][j],
            DIRECTIONS[j],
            distance,
            limit,
            active_previous,
        )
        criteria[name] = _describe_candidate(
            candidate["command"],
            candidate["approximate_effect"],
            duration,
            distance,
            limit,
            previous_limit,
        )
    return {
        "model": model,
        "state": state,
        "questions": {
            "action": {
                "type": "choice",
                "instructions": INSTRUCTIONS,
                "criteria": criteria,
            }
        },
    }


def decode_action(response, observation):
    answers = response.get("answers") if isinstance(response, dict) else None
    answer = answers.get("action") if isinstance(answers, dict) else None
    if not isinstance(answer, dict) or answer.get("type") != "choice":
        raise ValueError("Expected Choice answer for action")
    name = answer.get("choice")
    if not isinstance(name, str) or name not in ACTION_CHOICES:
        raise ValueError("Unknown speed/course choice")
    i, j = ACTION_CHOICES[name]
    return {
        "speed": observation["action_space"]["speeds_knots"][i],
        "course": observation["action_space"]["courses_deg"][j],
    }
