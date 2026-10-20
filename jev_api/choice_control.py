"""Route task and held navigation choices, expressed without signed-coordinate decoding."""

import math
from route_context import RouteContext
from tools.navigation_motion import delta_angle, predict_motion

DEFAULT_TRACKING_PREFERENCE_M = 0.25

SPEED_NAMES = (
    "reverse_full",
    "reverse_half",
    "stop",
    "forward_slow",
    "forward_quarter",
    "forward_half",
    "forward_three_quarters",
    "forward_full",
)
HEADING_NAMES = (
    "ahead",
    "ahead_left",
    "left",
    "behind_left",
    "behind",
    "behind_right",
    "right",
    "ahead_right",
)
ACTION_CHOICES = {
    f"{s}__{h}": {"speed_index": i, "heading_index": j}
    for i, s in enumerate(SPEED_NAMES)
    for j, h in enumerate(HEADING_NAMES)
}


def position_text(point):
    forward, left = point
    longitudinal = f"{abs(forward):.3f} m " + ("ahead" if forward >= 0 else "behind")
    lateral = f"{abs(left):.3f} m " + ("left" if left >= 0 else "right")
    return longitudinal + " and " + lateral


def speed_text(speed):
    return (
        "stationary"
        if abs(speed) < 1e-6
        else f'moving {"forward" if speed>0 else "backward"} at {abs(speed):.3f} m/s'
    )


def turn_text(yaw):
    return (
        "not rotating"
        if abs(yaw) < 1e-4
        else f'turning {"left" if yaw>0 else "right"} at {abs(yaw):.3f} deg/s'
    )


def heading_text(angle):
    return (
        "the current sampled bow direction"
        if abs(angle) < 1e-4
        else f'{abs(angle):.3f} degrees {"left" if angle>0 else "right"} of the current sampled bow'
    )


def speed_response(initial, target, transition_time, duration):
    if transition_time <= 0:
        return target * duration, target
    ramp = min(duration, transition_time)
    a = (target - initial) / transition_time
    return initial * ramp + 0.5 * a * ramp * ramp + target * (duration - ramp), (
        target if duration >= transition_time else initial + a * ramp
    )


BACKGROUND = (
    "You are the navigator of a boat in a two-dimensional simulator. A dashed reference route connects "
    "mandatory checkpoints in order. Your job is to follow this route, enter every checkpoint circle "
    "without passing it by, and finally stop inside the last circle. The route is the intended path "
    "between checkpoints, not merely decoration. The current checkpoint is the next mandatory gate, "
    "not a reason to shortcut a bend. No obstacles, wind, or sideways drift exist in this simulation. "
    "The world pauses while you decide. The boat executes one selected target speed and heading "
    "for the specified simulation duration before you receive a fresh observation."
)
INSTRUCTIONS = (
    "Route segments are ORDERED, even where they cross or run beside each other. "
    "ordered_route_ahead ends at the CURRENT mandatory checkpoint; follow that segment first. "
    "after_checkpoint_preview belongs to the NEXT segment: use it to anticipate braking and turning, "
    "not as a shortcut or a replacement for the current checkpoint. "
    "A checkpoint may require a near U-turn. After a checkpoint switches, the new route may be behind the bow; "
    "reduce speed and turn toward it rather than moving fast while the bow swings. "
    "When already outside the preferred tracking distance, that distance is temporarily unattainable: "
    "compare END distance reductions to recover toward the active line. Do not keep crawling parallel "
    "to an offset line merely to avoid a corrective turn. The near_route_direction points toward the "
    "first upcoming point on the active curve; distant checkpoints must not pull you across a bend. "
    "Use the longer SAME-command lookahead as a warning of imminent overshoot or leaving the route. "
    "Actual execution still lasts only the stated action duration and replans afterward. "
    "When a high-speed command would soon depart the curve, choose a slower approach before the turn. "
    "Aim to keep the whole movement within task.preferred_tracking_error_m of the reference line. "
    "This is a soft tracking preference, NOT the checkpoint arrival radius. "
    "Prefer a smaller-speed choice if needed to satisfy that tracking preference, while avoiding unnecessary crawling. "
    "When multiple choices have comparable tracking and end alignment, take the one with greater route advance. "
    "Also inspect END BOW ALIGNMENT and END YAW: an endpoint close to the line is not enough "
    "if the bow points across it or the boat is still rotating away. Prefer ending aligned with "
    "the local reference direction and with little residual rotation, while continuing useful progress. "
    "Slow before turns rather than alternate large left/right commands at high speed. "
    "Each candidate reports its geometric route error and along-route progress over the SAME action duration. "
    "Smaller route-error distances mean closer tracking; larger forward along-route advance means less completion time. "
    "Compare these jointly: do not crawl for a negligible error reduction, and do not accept large deviation merely for speed. "
    "Choose ONE combined target speed and target heading for the next action. "
    "First ensure the current checkpoint is not missed and the final stopping condition is respected. "
    "Follow the NEAR portion of the ordered reference route closely. Within those requirements, "
    "finish the route in as little SIMULATION TIME as possible: choose useful forward progress "
    "rather than unnecessary stops, crawling, or detours. Go fast on aligned straight sections; "
    "slow only as needed to negotiate a bend, correct deviation, enter a checkpoint, or stop. "
    "The candidate midpoint and endpoint describe curved motion in the CURRENT observation frame. "
    "A heading target is not an immediate movement direction. Account for existing rotation, "
    "the speed ramp, and the end yaw before choosing another turn. If the current checkpoint "
    "is behind, recover toward it rather than following the route beyond it. "
    "Checkpoint: enter its circle while moving; no parking. Final approach: enter and brake to "
    "remain inside. Final parking: target zero speed until actual speed satisfies the threshold. "
    "A stopped boat outside the final circle must approach, not repeatedly choose zero speed."
)


def build_payload(
    observation,
    model="jev-latest",
    context=None,
    run_key=None,
    tracking_preference_m=DEFAULT_TRACKING_PREFERENCE_M,
):
    if not math.isfinite(tracking_preference_m) or tracking_preference_m <= 0:
        raise ValueError("Tracking preference must be finite and positive")
    context = context or RouteContext()
    geometry = context.prepare(observation, run_key)
    task = observation["task"]
    boat = observation["boat"]
    sr = observation["response"]
    hr = observation["heading_response"]
    duration = observation["planning"]["action_duration_s"]
    target = geometry["target"]
    distance = math.hypot(*target["forward_left_m"])
    inside = distance <= target["arrival_radius_m"]
    final = task["next_waypoint_index"] == task["planWaypointNum"] - 1
    stage = (
        ("final_parking" if inside else "final_approach")
        if final
        else "checkpoint_tracking"
    )
    state = {
        "task": {
            "background": BACKGROUND,
            "stage": stage,
            "preferred_tracking_error_m": tracking_preference_m,
            "current_point_after_start": task["next_waypoint_index"],
            "total_points_after_start": task["planWaypointNum"] - 1,
            "current_goal": (
                "Stop inside the final circle with actual speed no more than "
                + str(task["stop_speed_mps"])
                + " m/s."
                if final
                else "Enter the current checkpoint circle; keep moving through it."
            ),
        },
        "boat": {
            "translation": speed_text(boat["speed_mps"]),
            "rotation": turn_text(boat["yaw_rate_deg_s"]),
        },
        "route": {
            "near_route_direction": heading_text(
                math.degrees(
                    math.atan2(
                        geometry["route"]["ahead_points_forward_left_m"][0][1],
                        geometry["route"]["ahead_points_forward_left_m"][0][0],
                    )
                )
            ),
            "remaining_along_route_to_checkpoint_m": geometry["route"][
                "remaining_to_checkpoint_m"
            ],
            "after_checkpoint_preview": [
                position_text(p)
                for p in geometry["route"]["after_checkpoint_forward_left_m"]
            ],
            "cross_track_distance_m": geometry["route"]["cross_track_distance_m"],
            "nearest_route_point": position_text(
                geometry["route"]["nearest_point_forward_left_m"]
            ),
            "ordered_route_ahead": [
                position_text(p)
                for p in geometry["route"]["ahead_points_forward_left_m"]
            ],
        },
        "target": {
            "position": position_text(target["forward_left_m"]),
            "distance_m": round(distance, 3),
            "arrival_radius_m": target["arrival_radius_m"],
            "inside_arrival_circle": inside,
        },
        "execution": {
            "duration_s": duration,
            "frame": "Every position is relative to the boat NOW: ahead/behind and left/right. The route and checkpoints are fixed in the world.",
            "speed_rule": f'A changed target starts constant acceleration immediately and reaches its target in {sr["speed_transition_time_s"]:.6g} s; zero time is instant. Keeping the same target continues its remaining ramp.',
            "current_speed_command": speed_text(sr["active_target_speed_mps"]),
            "speed_ramp_remaining_s": sr["remaining_transition_s"],
            "heading_rule": f'A changed heading target turns smoothly from the current yaw and arrives with zero yaw in {hr["transition_time_s"]:.6g} s; zero time is instant. The heading offset is set ONCE relative to the sampled bow. Keeping the same WORLD target continues its remaining turn.',
            "current_heading_command": heading_text(hr["active_target_offset_deg"]),
            "turn_remaining_s": hr["remaining_transition_s"],
        },
    }
    if sr["speed_transition_time_s"] <= 0:
        state["boat"].pop("translation")
        state["execution"].pop("current_speed_command")
        state["execution"].pop("speed_ramp_remaining_s")
    if hr["transition_time_s"] <= 0:
        state["boat"].pop("rotation")
        state["execution"].pop("current_heading_command")
        state["execution"].pop("turn_remaining_s")
    if not state["boat"]:
        state.pop("boat")
    criteria = {}
    for name, action in ACTION_CHOICES.items():
        speed = observation["action_space"]["speeds_mps"][action["speed_index"]]
        angle = observation["action_space"]["heading_offsets_deg"][
            action["heading_index"]
        ]
        st = (
            sr["remaining_transition_s"]
            if speed == sr["active_target_speed_mps"]
            else sr["speed_transition_time_s"]
        )
        same = abs(delta_angle(angle - hr["active_target_offset_deg"])) <= 0.0001
        ht = hr["remaining_transition_s"] if same else hr["transition_time_s"]
        turn_delta = hr["active_target_offset_deg"] if same else angle
        motion = predict_motion(
            boat["speed_mps"],
            speed,
            st,
            turn_delta,
            boat["yaw_rate_deg_s"],
            ht,
            duration,
            observation["fixed_dt_s"],
            include_path=True,
        )
        facts = context.motion_facts(motion["path"])
        advance = (
            f'{abs(facts["along_route_advance_m"]):.3f} m '
            + ("forward" if facts["along_route_advance_m"] >= 0 else "backward")
            + " along the ordered reference route"
        )
        future = predict_motion(
            boat["speed_mps"],
            speed,
            st,
            turn_delta,
            boat["yaw_rate_deg_s"],
            ht,
            duration * 2,
            observation["fixed_dt_s"],
            include_path=True,
        )
        future_facts = context.motion_facts(future["path"])
        alignment = delta_angle(motion["end_heading"] - facts["end_route_tangent_deg"])
        effect = (
            f'Over this action: route distance RMS {facts["route_error_rms_m"]:.3f} m, maximum {facts["route_error_max_m"]:.3f} m, ending {facts["route_error_end_m"]:.3f} m; '
            f'If the SAME command were held for {duration*2:.3f} s (hypothetical; actual action is shorter), route maximum distance would be {future_facts["route_error_max_m"]:.3f} m, ending distance {future_facts["route_error_end_m"]:.3f} m. '
            f"At end the bow is {abs(alignment):.1f} degrees "
            + ("LEFT" if alignment >= 0 else "RIGHT")
            + " of the local reference direction. "
            f'advance {advance}; current checkpoint circle {"entered" if facts["enters_checkpoint"] else "not entered yet"}; end distance to checkpoint {facts["end_checkpoint_distance_m"]:.3f} m. '
        )
        criteria[name] = (
            f"Target: {speed_text(speed)}; aim bow toward {heading_text(angle)}. "
            f'Halfway position: {position_text(motion["midpoint"])}. '
            f'End position: {position_text(motion["endpoint"])}. '
            f'End state: {speed_text(motion["end_speed"])}; bow toward {heading_text(motion["end_heading"])}; {turn_text(motion["end_yaw"])}. '
            + effect
            + "Choose when this curved movement follows the near route, passes the current checkpoint as required, and makes efficient progress."
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


def decode_action(response):
    answers = response.get("answers") if isinstance(response, dict) else None
    answer = answers.get("action") if isinstance(answers, dict) else None
    if not isinstance(answer, dict) or answer.get("type") != "choice":
        raise ValueError("Expected Choice answer for action")
    name = answer.get("choice")
    if not isinstance(name, str) or name not in ACTION_CHOICES:
        raise ValueError("Unknown speed/course choice")
    return dict(ACTION_CHOICES[name])
