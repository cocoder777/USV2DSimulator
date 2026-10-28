"""Common decision-envelope and physical-observation validation."""

import math
from decision_worker import IDENTITY_FIELDS
from tasks.registry import get_task
from tasks import stop


def valid_request(packet):
    if (
        not isinstance(packet, dict)
        or packet.get("type") not in ("decision_request", "stop_request")
    ):
        return False
    if any(
        not isinstance(packet.get(k), str) or not packet[k]
        for k in IDENTITY_FIELDS[:-1]
    ):
        return False
    if type(packet.get("tick")) is not int or packet["tick"] < 0:
        return False
    try:
        obs = packet["observation"]
        planning = obs["planning"]
        duration = planning["action_duration_s"]
        if "action_count" in planning or "actions" in obs:
            return False
        if (
            type(duration) not in (int, float)
            or not math.isfinite(duration)
            or duration <= 0
        ):
            return False
        boat, task, execution, space = (
            obs["boat"],
            obs["task"],
            obs["execution"],
            obs["action_space"],
        )

        def finite(v):
            return type(v) in (int, float) and math.isfinite(v)

        if not all(finite(boat[k]) for k in ("longitude", "latitude", "targetSpeed")):
            return False
        if not -180 <= boat["longitude"] <= 180 or not -90 <= boat["latitude"] <= 90:
            return False
        hull = boat.get("collision_radius_m")
        if not finite(hull) or hull <= 0:
            return False
        obstacles = obs.get("obstacles", [])
        if not isinstance(obstacles, list):
            return False
        ids = set()
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                return False
            identifier = obstacle.get("id")
            if not isinstance(identifier, str) or not identifier or identifier in ids:
                return False
            ids.add(identifier)
            obstacle_speed, obstacle_course = obstacle.get("targetSpeed"), obstacle.get("targetCourse")
            if obstacle_speed is not None and not finite(obstacle_speed):
                return False
            if obstacle_course is not None and (not finite(obstacle_course) or not 0 <= obstacle_course < 360):
                return False
            if (not all(finite(obstacle[k]) for k in ("longitude", "latitude", "radius_m"))
                or not -180 <= obstacle["longitude"] <= 180
                or not -90 <= obstacle["latitude"] <= 90 or obstacle["radius_m"] <= 0
                or not finite(obstacle["avoidance_radius_m"])
                or obstacle["avoidance_radius_m"] < obstacle["radius_m"]):
                return False
        course = boat["targetCourse"]
        if course is not None and (not finite(course) or not 0 <= course < 360):
            return False
        if course is None and abs(boat["targetSpeed"]) > 1e-5:
            return False
        if execution.get("motion_model") != "bounded_acceleration":
            return False
        if not all(finite(execution[k]) and execution[k] > 0 for k in (
            "max_acceleration_mps2", "max_deceleration_mps2", "max_yaw_rate_deg_s",
            "max_angular_acceleration_deg_s2", "fixed_dt_s", "hull_length_m", "hull_beam_m")):
            return False
        if (not finite(execution["heading_course_deg"]) or not 0 <= execution["heading_course_deg"] < 360
                or not finite(execution["yaw_rate_deg_s"])
                or abs(execution["yaw_rate_deg_s"]) > execution["max_yaw_rate_deg_s"] + 1e-4):
            return False
        if (not finite(execution["active_target_course_deg"]) or not 0 <= execution["active_target_course_deg"] < 360
                or not finite(execution["active_course_error_deg"])):
            return False
        ratio = duration / execution["fixed_dt_s"]
        if not 1 <= ratio <= 100000 or abs(ratio - round(ratio)) > 1e-3:
            return False
        speeds, courses = space["speeds_knots"], space["courses_deg"]
        if (
            len(speeds) != 8
            or len(courses) != 8
            or not all(finite(v) and v >= 0 for v in speeds)
            or not all(finite(v) and 0 <= v < 360 for v in courses)
        ):
            return False
        task_module = get_task(task["code"])
        operation = obs["operation"]
        if packet["type"] == "stop_request":
            return operation == "stop" and stop.validate_observation(obs, finite)
        if operation != "execute":
            return False
        return task_module.validate_observation(obs, finite)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
