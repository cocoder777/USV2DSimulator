"""56 joint acceleration choices; input formatting only, no local motion controller."""
import copy
import math


MODEL_STATE_FIELDS = (
    "coordinates",
    "planning",
    "boat",
    "target",
    "dynamics",
    "motion",
)

LINEAR_NAMES = (
    "negative_full",
    "negative_medium",
    "negative_gentle",
    "coast",
    "positive_gentle",
    "positive_medium",
    "positive_strong",
    "positive_full",
)

TURN_NAMES = (
    "clockwise_full",
    "clockwise_medium",
    "clockwise_gentle",
    "no_angular_input",
    "counterclockwise_gentle",
    "counterclockwise_medium",
    "counterclockwise_full",
)

LINEAR_SITUATIONS = (
    "Strongly reduce signed speed. Brake substantial forward motion when continued travel would "
    "overshoot the stop circle or carry the boat away while turning. With backward speed, this "
    "increases backward motion; continued braking past zero causes reverse travel.",
    "Clearly reduce signed speed when forward momentum needs substantial reduction during this "
    "interval. Do not choose merely because the target requires a final stop.",
    "Gently reduce signed speed when forward momentum needs a small reduction. It may cause "
    "unwanted backward motion if the boat is already slow; it is not a forward approach command.",
    "Apply no longitudinal acceleration; damping gradually slows existing forward or backward "
    "motion. Useful for coasting toward the target or turning with little translation.",
    "Gently increase signed speed for forward approach when modest thrust is enough, or gently "
    "reduce backward motion. Can sustain forward travel against damping. A nearby parking target "
    "does not justify thrust if existing forward speed already needs braking.",
    "Moderately increase signed speed to advance toward a target ahead or reduce backward motion. "
    "Account for damping and the travel needed before entering the target circle. Avoid adding "
    "forward momentum when approaching a nearby parking target too fast to stop inside it.",
    "Strongly increase signed speed for progress toward a distant target mostly ahead, or reduce "
    "backward motion. Avoid gaining forward momentum toward a target still behind the bow or "
    "when fast forward motion already threatens to overshoot a parking target.",
    "Maximally increase signed speed when strong forward acceleration toward a target ahead or "
    "strong braking of backward travel is appropriate. Continued input after backward speed reaches "
    "zero accelerates forward; avoid excessive acceleration when already near the stop circle.",
)

TURN_SITUATIONS = (
    "Target far RIGHT of the bow (negative left_m, or behind-right); strong clockwise turning is needed. "
    "Alternatively, target nearly straight AHEAD and substantial LEFT yaw needs strong braking.",
    "Target clearly RIGHT of the bow (negative left_m) and current right rotation is insufficient. "
    "Alternatively, target nearly straight AHEAD and LEFT yaw needs moderate braking.",
    "Target slightly RIGHT-front (small negative left_m, positive forward_m), needing a gentle clockwise "
    "correction. Alternatively, target nearly straight AHEAD and LEFT yaw needs gentle braking.",
    "Target straight AHEAD (positive forward_m, left_m near zero) AND yaw rate is near zero. No steering "
    "or rotational braking is needed.",
    "Target slightly LEFT-front (small positive left_m, positive forward_m), needing a gentle "
    "counterclockwise correction. Alternatively, target nearly straight AHEAD and RIGHT yaw needs gentle "
    "braking.",
    "Target clearly LEFT of the bow (positive left_m) and current left rotation is insufficient. "
    "Alternatively, target nearly straight AHEAD and RIGHT yaw needs moderate braking.",
    "Target far LEFT of the bow (positive left_m, or behind-left); strong counterclockwise turning is "
    "needed. Alternatively, target nearly straight AHEAD and substantial RIGHT yaw needs strong braking.",
)

COORDINATES = (
    "The boat is at (0,0), facing forward. target.forward_m is metres ahead (+) or behind (-); "
    "target.left_m is metres left (+) or right (-). Positive yaw rate and angular acceleration mean "
    "counterclockwise (left) rotation. These coordinates describe the state at the start of the "
    "requested action."
)

TASK = (
    "Pilot the boat into the CURRENT target circle using one combined longitudinal and angular "
    "acceleration input. Target coordinates are relative to the present boat: forward_m>0 is ahead; "
    "forward_m<0 is behind. left_m>0 is LEFT; left_m<0 is RIGHT. Positive speed travels forward; negative"
    " speed travels backward. Positive longitudinal input increases signed speed. Negative input brakes "
    "forward motion but makes backward motion faster. Positive input brakes backward motion. Positive "
    "yaw_rate_deg_s means already turning LEFT; negative means already turning RIGHT. Positive angular "
    "input increases LEFT rotation; negative increases RIGHT rotation. Inputs change yaw rate, not "
    "heading instantly. Steer toward the target side, but brake existing rotation before passing "
    "alignment. Zero angular input does not stop rotation instantly. A target behind calls for turning "
    "back rather than charging forward. Commands take effect immediately, but speed and yaw rate "
    "change over time; yaw rate changes heading, and speed moves the boat along that changing heading. "
    "The target is stationary in the world, so its relative coordinates change as the boat moves and turns. "
    "Account for damping even with zero input. Consider the travel and heading change throughout "
    "the action interval, not just the current side of the target. At high speed, turning alone "
    "may miss a nearby circle: reduce forward speed while steering when needed. Examples: "
    "forward_m=10,left_m=5,target ahead-left, requires left steering. forward_m=10,left_m=-5 requires "
    "right steering. forward_m=-3,left_m=4,target behind-left: turn left while limiting forward travel "
    "away. forward_m=10,left_m=0,yaw_rate_deg_s=20: brake LEFT rotation with NEGATIVE angular input. "
    "forward_m=10,left_m=0,yaw_rate_deg_s=-20: brake RIGHT rotation with POSITIVE angular input. "
    "Choose exactly one pair, held for the requested interval. Prefer approaching while moving forward: if currently "
    "moving backward, use positive longitudinal input to arrest backward travel before progressing "
    "forward. When a target is far to the side or behind, keep turning toward it; do not brake that turn "
    "merely because yaw is nonzero. Rotational braking is for impending alignment, not for a target still"
    " far from the bow direction."
)

CHECKPOINT_TASK = (
    "The CURRENT target is a CHECKPOINT. Enter its circle defined by target.arrival_radius_m; "
    "no stop is required. A subsequent decision supplies the next target after this checkpoint "
    "has been passed. Choose speed and turning that actually enter the circle; slow down if "
    "necessary to avoid missing it. "
)

PARKING_TASK = (
    "The CURRENT target is the FINAL PARKING REGION. Complete the task by being inside the "
    "circle defined by target.arrival_radius_m AND having abs(boat.speed_mps) <= "
    "target.stop_speed_mps at the same time. The stop-speed threshold applies INSIDE the circle, "
    "not throughout the approach. While far outside, make forward progress toward the circle; "
    "do not creep at the final stop speed or repeatedly brake an already slow boat. If current "
    "motion would stop well short of the circle, positive thrust is needed to keep approaching. "
    "As the circle gets close, reduce excess forward momentum early enough to stop inside it "
    "instead of overshooting and turning back. Brake when existing speed threatens to overshoot, "
    "including when the target is off to one side; steer and brake together when needed. "
    "Examples: forward_m=8, left_m=0, speed_mps=0.1 needs positive thrust to approach, not "
    "braking or coasting to a halt. forward_m=1.5, left_m=0, speed_mps=2 needs braking. "
    "forward_m=1.8, left_m=1.5, speed_mps=3.8 needs braking while steering, not more forward thrust. "
)

ACTION_CHOICES = {
    f"{linear}__{turn}": {"acceleration_index": a, "turn_index": t}
    for a, linear in enumerate(LINEAR_NAMES) for t, turn in enumerate(TURN_NAMES)
}


def make_instructions(duration, requires_stop):
    timing = (f"for the next {duration:.6g} seconds? "
              "The supplied state is measured at the start of this interval.")
    goal = PARKING_TASK if requires_stop else CHECKPOINT_TASK
    return (goal + TASK + " Which ONE combined longitudinal and angular acceleration command "
            "best achieves this target's completion condition " + timing)


def build_payload(observation, model="jev-latest"):
    # 深拷贝后转换坐标，原始观测保留给日志；参考路线不进入模型输入。
    state = {field: copy.deepcopy(observation[field]) for field in MODEL_STATE_FIELDS}
    boat, target = state["boat"], state["target"]
    x, y = boat.pop("position_xy_m")
    tx, ty = target.pop("position_xy_m")
    heading = math.radians(boat.pop("heading_deg"))
    dx, dy = tx - x, ty - y
    target["forward_m"] = dx * math.cos(heading) + dy * math.sin(heading)
    target["left_m"] = -dx * math.sin(heading) + dy * math.cos(heading)
    requires_stop = target.pop("requires_stop")
    if not requires_stop:
        target.pop("stop_speed_mps", None)
    state["coordinates"] = COORDINATES
    dyn, fractions = state["dynamics"], observation["action_space"]
    criteria = {}
    for key, action in ACTION_CHOICES.items():
        a, t = action["acceleration_index"], action["turn_index"]
        linear = fractions["acceleration_fractions"][a] * dyn["max_acceleration"]
        angular = fractions["turn_fractions"][t] * dyn["max_angular_acceleration_deg"]
        criteria[key] = (
            f"LONGITUDINAL situation: {LINEAR_SITUATIONS[a]} "
            f"ANGULAR situation: {TURN_SITUATIONS[t]} "
            f"Commands: longitudinal {linear:.4g} m/s^2; angular {angular:.4g} deg/s^2. "
            "Both situations must fit.")
    planning = state["planning"]
    questions = {
        "action": {
            "type": "choice",
            "instructions": make_instructions(planning["action_duration_s"], requires_stop),
            "criteria": dict(criteria),
        }
    }
    return {"model": model, "state": state, "questions": questions}


def decode_action(response):
    answers = response.get("answers") if isinstance(response, dict) else None
    # 只读取所需题目；额外回答不影响动作，非法选项仍会报错。
    if not isinstance(answers, dict) or "action" not in answers:
        raise ValueError("Expected action answer")
    answer = answers["action"]
    if not isinstance(answer, dict) or answer.get("type") != "choice":
        raise ValueError("Expected Choice answer for action")
    key = answer.get("choice")
    if not isinstance(key, str) or key not in ACTION_CHOICES:
        raise ValueError("Unknown joint action for action")
    return dict(ACTION_CHOICES[key])
