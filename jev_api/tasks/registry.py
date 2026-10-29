"""Single task-code registry; external codes never depend on file numbering."""

# Each registered module owns validate_params, validate_manifest,
# target_platform_id, adapt_observation, validate_observation, build_payload and
# status_params. A build_stop_payload hook is optional for target-specific facts.
# ACTION_CHOICES/COURSES_DEG belong to each task; shared STOP also uses DIRECTIONS.
# Navigation/following keep 64 choices; only accompaniment uses the finer 128 grid.

from tasks import task1_waypoint, distance_following, task3_accompany, stop

TASKS = {
    "trackingMode": task1_waypoint,
    "targetFollowing": distance_following,
    "accompanyMode": task3_accompany,
}


def get_task(code):
    try:
        return TASKS[code]
    except (KeyError, TypeError):
        raise ValueError("Unsupported task code") from None


def build_payload(observation, model):
    task = get_task(observation["task"]["code"])
    operation = observation["operation"]
    if operation == "stop":
        return stop.build_payload(observation, model, task)
    if operation != "execute":
        raise ValueError("Expected execute or stop operation")
    return task.build_payload(observation, model)
