"""Stop intent: preserve task identity while replacing its control objective."""

from geometry.coordinates import KNOT_MPS
from geometry.motion import predict_ground_motion, braking_facts, preview_duration
from geometry.obstacles import nearby_obstacles
from action_space import ACTION_CHOICES, DIRECTIONS
from tasks.presentation import position_text, describe_clearance, add_obstacle_state, AVOIDANCE_INSTRUCTIONS


def build_waypoint_stop_payload(observation, model="jev-latest"):
    """立即停车任务；复用原 128 个 Choice 和解码，不依赖航路点。"""
    boat = observation["boat"]
    duration = observation["planning"]["action_duration_s"]
    execution = observation["execution"]
    speed = boat["targetSpeed"] * KNOT_MPS
    course = boat["targetCourse"]
    criteria = {}
    obstacles = nearby_obstacles(observation)
    for name, (i, j) in ACTION_CHOICES.items():
        target_speed = observation["action_space"]["speeds_knots"][i] * KNOT_MPS
        target_course = observation["action_space"]["courses_deg"][j]
        motion = predict_ground_motion(
            speed, target_speed, target_course,
            execution, duration,
        )
        criteria[name] = (
            f"Set target ground speed {target_speed:.3f} m/s, absolute course "
            f"{target_course:.1f} degrees ({DIRECTIONS[j]}). "
            f"Approximate motion over {duration:.3f} s: endpoint "
            f"{position_text(motion['endpoint'])}; end speed {motion['end_speed']:.3f} m/s. "
            + ("Zero target speed brakes over time; the simulator ignores the requested course and cancels residual yaw within angular limits. "
               if target_speed == 0 else "")
        )
        if obstacles:
            future = predict_ground_motion(speed, target_speed, target_course,
                execution, preview_duration(execution, duration, speed), dt=max(0.1, execution["fixed_dt_s"]))
            criteria[name] = describe_clearance(motion, future, obstacles,
                boat["collision_radius_m"]) + criteria[name]
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
                "forecast": "Bounded-acceleration forecast from situation speed and measured heading/yaw; obstacles use current-position snapshots.",
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
                    "The simulator ignores the course of a zero-speed command and brakes at the configured maximum deceleration; "
                    "this does not instantly set actual speed to zero. "
                    "All original 128 choices remain available; no local controller chooses for you."
                ),
                "criteria": criteria,
            }
        },
    }
    payload["state"]["braking"] = braking_facts(speed, execution, action_duration=duration)
    if obstacles:
        add_obstacle_state(payload["state"], observation, obstacles)
        payload["questions"]["action"]["instructions"] = AVOIDANCE_INSTRUCTIONS + payload["questions"]["action"]["instructions"]
    return payload


def following_stop_objective(state, task):
    state["task"] = {"mode": "immediate_stop", "stop_speed_threshold_mps": task["stop_speed_mps"],
                     "objective": "Stop own boat. Distance-following and escape objectives are suspended."}
    state.pop("line_reference")
    instructions = (
        "IMMEDIATE STOP overrides the previous following mission. Choose ZERO target speed to brake own boat to rest "
        "and continue choosing zero until stopped. A nonzero target speed is not a stop command. "
        "At zero target speed, course is ignored; braking takes time under the configured deceleration. "
        "The moving target may pull away: this is expected and must NOT cause you to resume following, "
        "match its velocity or maintain the previous following distance. Distance-band and escape objectives are suspended. "
        "Only the current single action is executed; the long forecast is hypothetical. "
        "Physical collision facts remain visible; they do not turn this explicit stop request into a following task."
    )
    return instructions


def validate_observation(obs, finite):
    return finite(obs["task"]["stop_speed_mps"]) and obs["task"]["stop_speed_mps"] >= 0


def build_payload(obs, model, task_module):
    builder = getattr(task_module, "build_stop_payload", build_waypoint_stop_payload)
    return builder(obs, model)
