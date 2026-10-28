"""Single task-code registry; external codes never depend on file numbering."""

# Each registered module owns validate_params, validate_manifest,
# target_platform_id, adapt_observation, validate_observation, build_payload and
# status_params. A build_stop_payload hook is optional for target-specific facts.

from tasks import task1_waypoint, distance_following, stop

TASKS = {
    "trackingMode": task1_waypoint,
    "targetFollowing": distance_following,
}


def get_task(code):
    try:
        return TASKS[code]
    except (KeyError, TypeError):
        raise ValueError("Expected trackingMode or targetFollowing") from None


def build_payload(observation, model):
    task = get_task(observation["task"]["code"])
    operation = observation["operation"]
    if operation == "stop":
        return stop.build_payload(observation, model, task)
    if operation != "execute":
        raise ValueError("Expected execute or stop operation")
    return task.build_payload(observation, model)
