"""Jev chooses ground-speed/course commands from global navigation observations."""

import math
from tools.model_units import model_speed_units
from tools.waypoint_geometry import WaypointGeometry, KNOT_MPS
from tools.navigation_motion import predict_ground_motion
from tools.obstacle_geometry import (
    nearby_obstacles, describe_clearance, add_obstacle_state, AVOIDANCE_INSTRUCTIONS,
)

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
    "There is no prescribed path between points. Prefer a direct, nearly straight approach to the CURRENT waypoint "
    "with minimal unnecessary turning, rather than a curved, zigzag or axis-by-axis route. "
    "All positions and directions use FIXED geographic east/north axes, NEVER the bow or camera. "
    "No obstacles, wind or sideways drift are simulated. The world pauses while you decide. "
    "Each waypoint has a MAXIMUM permitted actual speed while passing its circle; this is only a ceiling, "
    "not a desired speed or a speed to match. Slower is allowed. "
    "A waypoint ceiling applies ONLY inside that waypoint's arrival circle, including until exit after passing. "
    "Outside active waypoint circles, actual and commanded speeds may exceed the next waypoint's ceiling. "
    "Subject to these local ceilings and the final stop requirement, minimize total simulated time to complete the route. "
    "You choose one nonnegative target ground speed and one ABSOLUTE course, clockwise from north. "
    "The simulator actuator turns the boat smoothly; a course command is not an instantaneous movement direction."
)
INSTRUCTIONS = (
    "Choose ONE command. Treat a predicted passing_speed_above_ceiling=true as a violation to avoid. "
    "Speed ceilings are mandatory and take priority over faster progress. An EXCEEDS warning must not be traded for speed. "
    "Subject to safe passing and stopping, prioritize a direct, nearly straight path toward the current waypoint. "
    "Use target.direct_course_deg_clockwise_from_north as the direct bearing, then choose an available course closest to it. "
    "Account for the turn transition: prefer a motion that settles toward the target, not merely a slightly nearer endpoint. "
    "Keep a useful course steady while it still leads into the arrival circle; do not chase the exact center with repeated left/right corrections. "
    "When turning is needed, make a deliberate correction, allow it to settle, then hold the useful course. "
    "Look ahead for lateral overshoot: do not hold a diagonal until one coordinate passes the target while the other remains far away. "
    "Correct toward the adjacent useful course BEFORE that overshoot, reducing speed if the turn cannot settle in time. "
    "Brake by reducing target SPEED, not by steering away from the waypoint. Do not choose an opposite or sideways "
    "course as a substitute for braking; keep aiming toward the arrival circle while slowing. "
    "Do not complete all east/west travel first and then all north/south travel when a diagonal approach is more direct. "
    "Avoid repeated course reversals or lateral excursions just to gain a small amount of one-action progress. "
    "Among safe choices, actively minimize remaining route completion time: accelerate for transit, brake before entry, "
    "pass intermediate circles without unnecessary stopping, and accelerate again after exiting. "
    "Do NOT use the next waypoint's passing ceiling as a speed cap for the entire leg. "
    "Outside active circles, choose higher available transit speeds when enough distance remains "
    "to slow to the next waypoint's passing ceiling BEFORE entry, with margin. "
    "If excess is already unavoidable from current motion, brake strongly to reduce it rather than accelerate. "
    "Never accelerate hard near a circle merely because this single action stops short of entering it. "
    "Use the longer same-command preview to anticipate the following action. "
    "On final approach use a slow controlled entry under its ceiling, then stop. "
    "Once braking for final entry, do not accelerate again merely to reach the center faster. "
    "A direct path never justifies a speed violation: if a candidate predicts EXCEEDS, prefer a safe braking action "
    "even if its distance reduction is smaller or temporarily negative. "
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
    "A useful transit command need not reach the circle in this action. Compare safe candidates' distance reduction "
    "over the same action duration, while accounting for the braking needed afterward. "
    "Prefer faster useful progress when braking remains feasible; choosing a slower available speed needs a reason "
    "such as an active circle ceiling, braking distance, turning uncertainty or final parking. "
    "Travel fast when far enough away, aligned, and able to brake before the next circle; slow for turns or nearby targets. "
    "Only eight compass courses are available, so an exact straight bearing may be unavailable. "
    "Use adjacent courses that bracket the direct bearing when necessary, with timely small corrections rather than long "
    "off-bearing runs followed by large recovery turns. Hold a course only while its predicted motion remains useful. "
    "A course change after passing a waypoint is appropriate for the next leg; do not turn early toward a future point. "
    "Inspect the predicted endpoint distance and entry facts, but these are APPROXIMATIONS from ground velocity: "
    "bow direction, yaw rate and actuator progress are unavailable. In particular startup and sharp turns may differ. "
    "If the waypoint is behind the current movement, slow before turning back. "
    "When very close or already inside the final circle, prefer braking to zero over another accelerating move. "
    "All candidates remain available: no local controller selects, ranks or filters your choice."
)


# Navigation-specific wording: the immediate-stop request keeps its existing instructions.
NAVIGATION_AVOIDANCE_INSTRUCTIONS = (
    "HIGHEST PRIORITY: proactively steer around moving obstacles BEFORE getting close. "
    "If the direct approach would reduce hull clearance below 1 m, start a bypass toward a clear side of the obstacle now. "
    "Keep a useful bypass course steady; do not turn back toward the waypoint before passing clear of the obstacle. "
    "Aim for at least 1 m extra hull clearance. If already closer, actively increase clearance rather than creep forward or wait beside it. "
    "Choose a short safe detour that also advances toward the CURRENT waypoint when possible; a temporary increase in waypoint distance is allowed for safety. "
    "Once clear, resume the current waypoint. These avoidance requirements override direct-course preferences below. "
    "Obstacles keep moving when you stop; forecasts freeze their current positions even when movement speed and course are supplied. "
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
    obstacles=(),
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
    if obstacles:
        candidate["approximate_effect"]["obstacle_description"] = describe_clearance(
            motion, future, obstacles, boat.get("collision_radius_m", 0.5))
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
    text = ""
    text += effect.get("obstacle_description", "")
    if effect["passing_speed_above_ceiling"] or effect.get("previous_circle_speed_above_ceiling", False):
        text += "SPEED VIOLATION predicted during this action. Avoid; if unavoidable, brake to minimize excess. "
    text += (
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
    if "obstacle_description" in effect:
        return text + (
            "Choose this movement to start or continue an EARLY bypass on a clear side of the obstacle, maintaining at least 1 m extra hull clearance when feasible. "
            "If already too close, actively increase end clearance. Continue the bypass until clear instead of turning back toward the waypoint too soon. "
            "Among safe bypasses, make progress toward the current waypoint and keep the detour short; temporary loss of progress is allowed for safety. "
            "Respect active circle speed ceilings among safe movements."
        )
    text += (
        "Choose when this movement respects active circle ceilings, leaves room to brake before entry, "
        "and makes direct progress with little lateral overshoot or corrective turning. "
        "Transit above the next circle's ceiling is allowed outside active circles; "
        "this action need not reach the circle. Speed safety and final braking take priority over path and time."
    )
    return text


@model_speed_units
def build_stop_payload(observation, model="jev-latest"):
    """立即停车任务；复用原 64 个 Choice 和解码，不依赖航路点。"""
    boat = observation["boat"]
    duration = observation["planning"]["action_duration_s"]
    execution = observation["execution"]
    speed = abs(boat["targetSpeed"]) * KNOT_MPS
    course = boat["targetCourse"]
    criteria = {}
    obstacles = nearby_obstacles(observation)
    for name, (i, j) in ACTION_CHOICES.items():
        target_speed = observation["action_space"]["speeds_knots"][i] * KNOT_MPS
        target_course = observation["action_space"]["courses_deg"][j]
        motion = predict_ground_motion(
            speed, course, target_speed, target_course,
            execution["speed_transition_time_s"],
            execution["course_transition_time_s"], duration,
        )
        criteria[name] = (
            f"Set target ground speed {target_speed:.3f} m/s, absolute course "
            f"{target_course:.0f} degrees ({DIRECTIONS[j]}). "
            f"Approximate motion over {duration:.3f} s: endpoint "
            f"{position_text(motion['endpoint'])}; end speed {motion['end_speed']:.3f} m/s. "
            + ("Zero target speed brakes over time; the simulator keeps the received bow direction. "
               if target_speed == 0 else "")
        )
        if obstacles:
            future = predict_ground_motion(speed, course, target_speed, target_course,
                execution["speed_transition_time_s"], execution["course_transition_time_s"], duration * 2)
            criteria[name] = describe_clearance(motion, future, obstacles,
                boat.get("collision_radius_m", 0.5)) + criteria[name]
    payload = {
        "model": model,
        "state": {
            "task": {
                "stage": "immediate_stop",
                "background": "A stop request has replaced waypoint navigation. Stop the boat as soon as possible at its present location.",
                "stop_speed_threshold_mps": observation["task"]["stop_speed_mps"],
            },
            "boat": {
                "actual_speed_mps": round(speed, 6),
                "movement_course_deg_clockwise_from_north": course,
                "course_available": course is not None,
            },
            "execution": {
                "duration_s": duration,
                **execution,
                "frame": "Fixed east/north; course 0=N,90=E,180=S,270=W.",
                "forecast": "Approximate ground-velocity forecast; bow direction, yaw rate and actuator progress are unavailable.",
            },
        },
        "questions": {
            "action": {
                "type": "choice",
                "instructions": (
                    "Choose ONE command to stop immediately, minimizing further travel. "
                    "Choose a zero target speed to brake and keep it zero once stopped. "
                    "Do not approach any waypoint or final circle, resume navigation, or accelerate to make progress. "
                    "Stopping has no position or arrival-circle requirement. "
                    "The simulator ignores the course of a zero-speed command and brakes over its speed transition time; "
                    "this does not instantly set actual speed to zero. "
                    "All original 64 choices remain available; no local controller chooses for you."
                ),
                "criteria": criteria,
            }
        },
    }
    if obstacles:
        add_obstacle_state(payload["state"], observation, obstacles)
        payload["questions"]["action"]["instructions"] = AVOIDANCE_INSTRUCTIONS + payload["questions"]["action"]["instructions"]
    return payload


@model_speed_units
def build_payload(observation, model="jev-latest"):
    if observation["task"].get("mode") == "immediate_stop":
        return build_stop_payload(observation, model)
    geometry = WaypointGeometry(observation)
    obstacles = nearby_obstacles(observation)
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
            "direct_course_deg_clockwise_from_north": (
                round(math.degrees(math.atan2(target[0], target[1])) % 360, 3)
                if distance > 1e-6 else None
            ),
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
    if obstacles:
        add_obstacle_state(state, observation, obstacles)
        state["task"]["background"] = BACKGROUND.replace(
            "No obstacles, wind or sideways drift are simulated.",
            "Moving circular obstacles are simulated; wind and sideways drift are not.")
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
            obstacles,
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
                "instructions": (NAVIGATION_AVOIDANCE_INSTRUCTIONS if obstacles else "") + INSTRUCTIONS,
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
