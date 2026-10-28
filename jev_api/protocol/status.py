"""Status JSON construction; task-specific details belong to tasks/."""

from tasks.registry import TASKS
from tasks import task1_waypoint


def status_content(task, status, description, contexts, result=None, control_stamp=None):
    key = task["simId"], task["id"]
    context = task.get("params", contexts.get(key, {}))
    # Rejected unknown codes must still receive the original correlated failure
    # response. This fallback does not admit them to the decision registry.
    task_module = TASKS.get(task["code"], task1_waypoint)
    params = task_module.status_params(context)
    local = task.get("unity_only", {})
    if local.get("decision_id"):
        params["unity_only"] = {k: local[k] for k in ("decision_id", "tick")}
    if result is not None:
        params["unity_only"] = {k: result[k] for k in ("decision_id", "tick")}
        if control_stamp is not None:
            params["unity_only"]["control_timestamp"] = control_stamp
    content = {k: task[k] for k in ("simId", "id", "code")}
    content.update(status=status, description=description, params=params)
    return content
