"""Public model-request entry, shared 128 Choices and action decoding."""

from action_space import ACTION_CHOICES, DIRECTIONS
from tasks.registry import build_payload as build_task_payload
from tasks import stop
from tools.model_units import model_speed_units


@model_speed_units
def build_payload(observation, model="jev-latest"):
    return build_task_payload(observation, model)


@model_speed_units
def build_stop_payload(observation, model="jev-latest"):
    from tasks.registry import get_task
    return stop.build_payload(observation, model, get_task(observation["task"]["code"]))


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
