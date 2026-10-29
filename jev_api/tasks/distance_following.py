"""Existing line-of-sight distance task; not the customer followMode."""

import math
from action_space import ACTION_CHOICES, COURSES_DEG
from geometry.coordinates import KNOT_MPS, xy
from geometry.motion import predict_ground_motion, preview_duration, braking_facts
from geometry.obstacles import nearby_obstacles, clearance_facts
from geometry.relative_motion import direction, velocity, relative_facts, line_reference_facts, course_error, moving_range
from tasks.stop import following_stop_objective

# Shared schema/rules replace prose repeated for each of the 64 choices.
CANDIDATE_SCHEMA = {
    "units": "Distances m; angles deg clockwise from north; yaw deg/s. Speed and range_rate use state.units.speed (m/s if units absent).",
    "speed_course": "speed_mps/speed_knots and course_deg are commands; end_motion contains actual end speed/course/yaw. A zero-speed command ignores course and brakes over time.",
    "range_m": "[action minimum, action maximum, action end], centre-to-centre; minima include the current state.",
    "range_rate": "End relative radial speed: negative closing, positive opening.",
    "target_hull_clearance_m": "[action min, preview min] range minus combined hull radii; <=0 means contact.",
    "radial_braking_m": "[closing-speed braking distance, end range minus minimum allowed range minus that distance]. Assumes aligned braking/constant target velocity; not a turning safety guarantee.",
    "reference_error_m": "[action end, preview end] absolute deviation from reference distance; current error is in line_reference. Midpoint is soft.",
    "preview_range_m": "[minimum, maximum, end] if this command is held for forecast.preview_duration_s; hypothetical, not the executed action.",
    "turn_error_deg": "[action-end absolute bow-to-target error, commanded-course-to-current-target error]; current signed error is in range_recovery.",
    "obstacle_hull_clearance_m": "[action min, action end, preview min, preview end], subtracting own hull and obstacle physical radius.",
    "obstacle_safety_clearance_m": "[action min, preview min, preview end], subtracting own hull and obstacle safety radius. Obstacles frozen at current positions.",
}
WARNING_MEANINGS = {
    "TARGET_COLLISION": "Collision with tracked target predicted during action OR held-command preview.",
    "ALREADY_BELOW_MIN": "All choices share an already-too-close start; this common minimum is not a new violation.",
    "SEPARATION_INCREASING": "Action end opens range: recovery when already too close.",
    "SEPARATION_DECREASING": "Short action closes range; compare end range rate and longer recovery under inertia.",
    "SEPARATION_UNCHANGED": "No separation recovery during this action.",
    "MIN_RANGE_ACTION": "Minimum following distance violation during this action predicted.",
    "MIN_RANGE_PREVIEW_ONLY": "Holding command beyond this action may violate minimum; action itself stays above minimum.",
    "NO_REORIENTATION": "Zero speed does not initiate the requested turn; course is ignored.",
    "COMMAND_POINTS_AWAY": "Command points away from target while recovering excessive or impending excessive range.",
    "BRAKING": "Not instant stopping; target continues moving. Zero own speed is not retreat from an incoming target.",
    "ZERO_TARGET_SPEED": "Brake to stop; keep zero once stopped; course ignored.",
    "NONZERO_NOT_STOP": "Nonzero target speed is not a stop command.",
    "OBSTACLE_SAFETY": "Predicted safety-boundary intrusion: choose an early safe bypass.",
    "OBSTACLE_COLLISION": "Predicted hull intersection with an obstacle.",
}


def build_payload(obs, model="jev-latest"):
    task, boat, target, execution = obs["task"], obs["boat"], obs["target"], obs["execution"]
    duration = obs["planning"]["action_duration_s"]
    speed = boat["targetSpeed"] * KNOT_MPS
    position = xy((target["longitude"], target["latitude"]), (boat["longitude"], boat["latitude"]))
    target_velocity = velocity(target["targetSpeed"], target["targetCourse"])
    forward = direction(task["reference_course_deg"])
    lower, upper = task["minFollowingDistanceM"], task["maxFollowingDistanceM"]
    reference = (lower + upper) / 2
    own_velocity = velocity(boat["targetSpeed"], boat["targetCourse"])
    relative = relative_facts(position, own_velocity, target_velocity, forward)
    current = {k: relative[k] for k in ("distance_m", "range_rate_mps")}
    unit = [v / current["distance_m"] for v in position] if current["distance_m"] > 1e-9 else [0, 0]
    current["target_radial_velocity_mps"] = sum(v * u for v, u in zip(target_velocity, unit))
    current["own_radial_velocity_mps"] = sum(v * u for v, u in zip(own_velocity, unit))
    current["target_approaching_own_position"] = current["target_radial_velocity_mps"] < -.1
    line_reference = line_reference_facts(position, reference)
    stopping = obs["operation"] == "stop"
    too_far = not stopping and current["distance_m"] > upper
    bearing = line_reference["line_of_sight_course_deg"]
    heading_error = course_error(bearing, execution["heading_course_deg"]) if bearing is not None else 0
    opening = current["range_rate_mps"]
    # Time needed to turn from an outward heading to an inward one. Include
    # yaw acceleration/inertia and one decision interval; this is not a path plan.
    turn_to_inward = (max(0, abs(heading_error) - 90) / execution["max_yaw_rate_deg_s"]
                      + (execution["max_yaw_rate_deg_s"] + abs(execution["yaw_rate_deg_s"]))
                      / execution["max_angular_acceleration_deg_s2"])
    time_to_upper = max(0, upper-current["distance_m"]) / opening if opening > .1 else None
    early_reorient = (not stopping and reference - 1e-6 <= current["distance_m"] <= upper
                      and abs(heading_error) > 90 and time_to_upper is not None
                      and time_to_upper <= turn_to_inward + duration)
    # Geometric phase only: no candidate selection, ranking, filtering or actuator override.
    reorient = (too_far and abs(heading_error) > 90) or early_reorient
    horizon = preview_duration(execution, duration, speed)
    combined = boat["collision_radius_m"] + target["radius_m"]
    obstacles = nearby_obstacles(obs)
    state = {
        "task": {"mode": "immediate_stop" if stopping else "target_following", "guidance": "line_of_sight_distance", "min_distance_m": lower, "max_distance_m": upper,
                 "reference_distance_m": reference,
                 "distance_definition": "Centre-to-centre Euclidean range. Reference lies on the current own-boat/target line, on the target side facing own boat. No rear-position or lateral-offset objective.",
                 "stage": "too_close" if current["distance_m"] < lower else "too_far" if current["distance_m"] > upper else "in_band",
                 "completion": "Entering the band does not finish the mission. Continue until explicit end or stop."},
        "boat": {"actual_speed_mps": speed, "movement_course_deg": boat["targetCourse"]},
        "target": {"platform_id": task["targetPlatformId"], "east_m": position[0], "north_m": position[1],
                   "actual_speed_mps": target["targetSpeed"] * KNOT_MPS, "movement_course_deg": target["targetCourse"],
                   "stationary": task["target_stationary"],
                   "hull_radius_m": target["radius_m"]},
        "relative": current,
        "line_reference": line_reference,
        "execution": dict(execution, duration_s=duration),
        "forecast": {"model": "Target constant current velocity; own bounded acceleration/yaw. Future target commands are unknown.",
                     "preview_duration_s": horizon, "preview_step_s": max(.1, execution["fixed_dt_s"]),
                     "warning": "Long preview assumes holding this action, not a promised trajectory. Sudden turns/stops invalidate prediction. No interception point is computed."},
        "braking": braking_facts(speed, execution, action_duration=duration),
        "obstacles": obstacles,
        "candidate_schema": CANDIDATE_SCHEMA,
        "warning_meanings": WARNING_MEANINGS,
    }
    # With a zero speed ceiling this equals distance_to_stop_m exactly.
    state["braking"].pop("distance_to_ceiling_m")
    if too_far or early_reorient:
        state["range_recovery"] = {
            "phase": "reorient_before_catchup" if reorient else "close_range",
            "target_bearing_deg": bearing,
            "signed_heading_error_deg": heading_error,
            "own_motion_points_away": current["own_radial_velocity_mps"] < -.1,
            "yaw_rate_deg_s": execution["yaw_rate_deg_s"],
            "turning_toward_target_bearing": heading_error * execution["yaw_rate_deg_s"] > 0,
            "meaning": "Recover orientation before accelerating away. Short-action range can increase during a useful bounded turn; keep a safe turn direction while reorienting. Not an interception point or selected command.",
        }
        if early_reorient:
            state["range_recovery"].update(trigger="opening_threatens_maximum_before_turn",
                time_to_maximum_at_current_range_rate_s=time_to_upper,
                estimated_time_to_inward_heading_s=turn_to_inward,
                warning="Current range is still allowed. Start a safe recovery turn before it exceeds maximum; estimates assume current range rate/bearing and are not guarantees.")
    criteria = {}
    for name, (i, j) in ACTION_CHOICES.items():
        commanded_speed = obs["action_space"]["speeds_knots"][i] * KNOT_MPS
        course = obs["action_space"]["courses_deg"][j]
        motion = predict_ground_motion(speed, commanded_speed, course, execution, duration, route_details=False)
        motion["sample_dt_s"] = execution["fixed_dt_s"]
        step = max(.1, execution["fixed_dt_s"])
        future = predict_ground_motion(speed, commanded_speed, course, execution, horizon, dt=step, route_details=False)
        future["sample_dt_s"] = step
        minimum, maximum, end_delta = moving_range(motion, position, target_velocity, duration)
        preview_min, preview_max, future_delta = moving_range(future, position, target_velocity, horizon)
        warnings = []
        # Rounded at the same m/s precision as the previous prose renderer.
        facts = {"warnings": warnings, "speed_mps": round(commanded_speed, 3), "course_deg": course,
                 "end_motion": {"speed_mps": round(motion["end_speed"], 3)},
                 "target_hull_clearance_m": [minimum-combined, preview_min-combined]}
        if stopping:
            warnings.append("ZERO_TARGET_SPEED" if commanded_speed == 0 else "NONZERO_NOT_STOP")
        else:
            end_velocity = tuple(motion["end_speed"] * a for a in direction(motion["end_course_deg"]))
            end = relative_facts(end_delta, end_velocity, target_velocity, forward)
            future_distance = math.hypot(*future_delta)
            closing = max(0.0, -end["range_rate_mps"])
            braking_distance = closing * closing / (2 * execution["max_deceleration_mps2"])
            facts.update(end_motion=dict(facts["end_motion"], course_deg=motion["end_course_deg"], yaw_deg_s=motion["end_yaw_rate_deg_s"]),
                range_m=[minimum, maximum, end["distance_m"]], range_rate_mps=round(end["range_rate_mps"], 3),
                radial_braking_m=[braking_distance, end["distance_m"]-lower-braking_distance],
                reference_error_m=[abs(end["distance_m"]-reference), abs(future_distance-reference)],
                preview_range_m=[preview_min, preview_max, future_distance])
            if minimum <= combined or preview_min <= combined:
                warnings.append("TARGET_COLLISION")
            if current["distance_m"] < lower:
                warnings.append("ALREADY_BELOW_MIN")
                warnings.append("SEPARATION_INCREASING" if end["distance_m"] > current["distance_m"] + .001 else
                                "SEPARATION_DECREASING" if end["distance_m"] < current["distance_m"] - .001 else "SEPARATION_UNCHANGED")
            elif minimum < lower:
                warnings.append("MIN_RANGE_ACTION")
            elif preview_min < lower:
                warnings.append("MIN_RANGE_PREVIEW_ONLY")
            if too_far or early_reorient:
                end_bearing = math.degrees(math.atan2(end_delta[0], end_delta[1])) % 360
                command_error = course_error(bearing, course)
                facts["turn_error_deg"] = [abs(course_error(end_bearing, motion["end_course_deg"])), abs(command_error)]
                if commanded_speed == 0:
                    warnings.append("NO_REORIENTATION")
                elif abs(command_error) > 90:
                    warnings.append("COMMAND_POINTS_AWAY")
            if commanded_speed == 0:
                warnings.append("BRAKING")
        if obstacles:
            clearance = clearance_facts(motion, future, obstacles, boat["collision_radius_m"])
            facts.update(clearance)
            if min(clearance["obstacle_safety_clearance_m"][:2]) < 0:
                warnings.append("OBSTACLE_SAFETY")
            if min(clearance["obstacle_hull_clearance_m"][::2]) <= 0:
                warnings.append("OBSTACLE_COLLISION")
        criteria[name] = facts
    instructions = (
        "Choose one speed/course action for safe LINE-OF-SIGHT DISTANCE FOLLOWING of the designated moving target. "
        "Priority: avoid hull collisions with target and obstacles; protect minimum following range with EARLY braking; "
        "recover/maintain the allowed distance band with small relative range velocity and avoid oscillation. "
        "Any relative bearing is allowed: side-by-side or ahead is not an error. Do not chase a position behind target heading. "
        "Do not intercept or collide. Negative range rate means approaching: act BEFORE crossing minimum. "
        "Use active retreat ONLY when already too close or action/turn/braking forecasts show a threat to minimum range. "
        "In that condition, if target heads toward own boat, turn/move away or take a safe lateral escape that opens separation. "
        "Zero own speed does NOT stop an incoming target; near minimum do not repeatedly brake/wait while it closes. "
        "An approaching target at a safe or excessive distance is NOT by itself a reason to flee. "
        "Because turning and acceleration take time, start an escape EARLY; compare short-action clearance and longer recovery. "
        "The directly-away bearing is geometric information, not a forced command: obstacles and turn inertia may require another escape side. "
        "Do not stop just because range is inside the band: retain appropriate motion as target moves. "
        "When reference distance equals current range, the reference point equals own position; this does NOT mean park there. "
        "Prefer a stable near-zero relative velocity within the band; reference midpoint is a soft preference, not a mandatory point. "
        "Use action-period forecasts first and hypothetical long previews for early braking/turning; long previews grow uncertain. "
        "A long-preview minimum violation does NOT mean this single short action violates minimum. "
        "If too far away, safely close range NOW and replan after this action; target approaching can HELP recovery. "
        "Do not move farther away merely because target_radial_velocity is negative. "
        "A positive radial braking margin with aligned motion "
        "indicates room to brake back toward matching speed later; do not hold matching speed forever outside maximum "
        "just because holding a catch-up command for the entire long preview would eventually get too close. "
        "If target stops, settle safely within the band. No particular rear direction is required. "
        "If already outside a bound, choose a safe recovery; every minimum includes the initial state, so compare end range/range rate. "
        "Other obstacle forecasts freeze their current positions. Preserve their configured hull/safety clearance; safe detours may temporarily lose spacing. "
        "Code only computes facts; all original 64 choices remain available to you."
    )
    if stopping:
        instructions = following_stop_objective(state, task)
    elif current["distance_m"] < lower:
        instructions = (
            "CURRENTLY TOO CLOSE: actively INCREASE separation to at least the minimum. "
            "Every choice includes the SAME already-too-close starting distance; that common initial minimum is unavoidable, "
            "not a reason to refuse recovery. Compare INCREASING versus NO SEPARATION RECOVERY and the end range rate. "
            "Matching target velocity while too close freezes the violation indefinitely and is not recovery. "
            "When target approaches your position, actively MOVE AWAY; parking lets it keep approaching. "
            "Choose nonzero movement that opens range, using a safe turn/escape side under the physical limits; "
            "braking may help a turn but does not by itself solve an incoming-target encounter. "
            "When target is moving away instead, slowing down can let it gain separation. "
            "Resume range-stable motion only once safe spacing is restored. Preserve physical collision clearance. "
        ) + instructions
    elif reorient:
        instructions = (
            ("PREDICTED LOSS OF MAXIMUM RANGE: range is still in band but opening faster than the bow can turn inward. "
             "Start a safe reorientation NOW rather than waiting to exceed maximum. Protect minimum range while doing so. "
             if early_reorient else "TOO FAR AND MISALIGNED: first REORIENT toward the target bearing, then close range. ")
            +
            "Do not accelerate along a heading that points away from the target. "
            "Choose a safe NONZERO-speed turning command; zero-speed course is ignored and cannot turn the bow. "
            "Use a speed that keeps the turning path safe, rather than speeding up to compensate for wrong orientation. "
            "The lowest nonzero speed is NOT mandatory: do not crawl when a faster turn is safe. "
            "Once the bow points toward the target (heading error at most 90 deg), recover closing speed promptly while finishing alignment. "
            "A useful turn can initially INCREASE range just as straight-away motion does: the bow turns only a few degrees in one action. "
            "Compare commanded-course error, end heading error/yaw and the longer recovery; do not reject all turns because one-second range increases. "
            "Keep a useful safe turn direction through successive actions while reorienting; do not repeatedly cancel/reverse the turn "
            "for tiny short-action range gains. Account for measured yaw inertia. "
            "Once oriented toward the target, accelerate as needed to close range and brake before minimum. "
            "Target bearing is current geometry, not an interception destination. Collision/obstacle safety overrides this recovery phase. "
        ) + instructions
    instructions += " Each criterion is a JSON facts table. Read candidate_schema for columns/units and warning_meanings for every warning; all 64 choices remain available."
    return {"model": model, "state": state, "questions": {"action": {"type": "choice", "instructions": instructions, "criteria": criteria}}}


def validate_observation(obs, finite):
    task = obs["task"]
    hull = obs["boat"]["collision_radius_m"]
    target = obs["target"]
    return (all(finite(target[k]) for k in ("longitude", "latitude", "targetSpeed", "radius_m"))
            and -180 <= target["longitude"] <= 180 and -90 <= target["latitude"] <= 90
            and 0 <= target["targetSpeed"] <= 200 and target["radius_m"] > 0
            and (target["targetCourse"] is None and target["targetSpeed"] == 0
                 or finite(target["targetCourse"]) and 0 <= target["targetCourse"] < 360)
            and all(finite(task[k]) for k in ("minFollowingDistanceM", "maxFollowingDistanceM", "reference_course_deg"))
            and target["radius_m"] + hull < task["minFollowingDistanceM"] < task["maxFollowingDistanceM"]
            and 0 <= task["reference_course_deg"] < 360)


def validate_params(params):
    lower, upper = params.get("minFollowingDistanceM"), params.get("maxFollowingDistanceM")
    if (type(params.get("targetPlatformId")) is not int or not 0 <= params["targetPlatformId"] <= 65535
            or params["targetPlatformId"] == params["platCode"]
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (lower, upper))
            or not 0 < lower < upper):
        raise ValueError("Invalid target following task")


def target_platform_id(params):
    return params["targetPlatformId"]


def validate_manifest(params, radii, expected):
    target_id = params["targetPlatformId"]
    if target_id not in expected or params["minFollowingDistanceM"] <= radii[str(target_id)] + radii[str(params["platCode"])]:
        raise ValueError("Missing target or following minimum inside combined hulls")


def adapt_observation(params, local, records, obs, state):
    target = next(r for r in records if r["platformId"] == params["targetPlatformId"])
    speed, course = target["targetSpeed"], target["targetCourse"]
    if speed is None or speed < 0 or (speed > 1e-5 and course is None):
        raise ValueError("Unknown or unsupported target motion")
    speed_mps = speed * KNOT_MPS
    stationary = speed_mps < (0.2 if state.get("stationary", False) else 0.1)
    reference_course = state.get("course")
    if not stationary and course is not None:
        reference_course = course
    if reference_course is None:
        boat = obs["boat"]
        delta = xy((target["longitude"], target["latitude"]), (boat["longitude"], boat["latitude"]))
        reference_course = math.degrees(math.atan2(delta[0], delta[1])) % 360
    state.update(stationary=stationary, course=reference_course)
    obs["target"] = dict(target, radius_m=local["collision_radii_m"][str(params["targetPlatformId"])])
    obs["task"] = dict(params, code="targetFollowing", reference_course_deg=reference_course,
                       target_stationary=stationary)


def build_stop_payload(obs, model="jev-latest"):
    # Preserve target-motion/collision facts and the existing compact stop choices.
    return build_payload(obs, model)


def status_params(context, observation=None):
    return {"targetPlatformId": context.get("targetPlatformId")}
