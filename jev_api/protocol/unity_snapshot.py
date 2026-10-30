"""Unity snapshot barrier and task-independent observation assembly."""

import math
import time
from geometry.coordinates import KNOT_MPS
from protocol.frames import STOP
from tasks.registry import get_task
from action_space import SPEED_FRACTIONS


class UnitySnapshotAdapter:
    """汇齐同次快照；任务业务适配由注册的任务模块处理。"""
    def __init__(self, speed_unit="knots"):
        self.speed_unit = speed_unit
        self.situations = {}
        self.waiting = {}
        self.tasks = {}
        self.retired = set()
        self.current = None
        self.seen = set()
        self.last_tick = -1
        self.task_state = {}
        self.last_observations = {}

    def add_situation(self, record):
        key = record["timestamp"], record["platformId"]
        previous = self.situations.get(key)
        if previous is not None and previous[0] != record:
            raise ValueError("Conflicting entity in the same snapshot")
        now = time.monotonic()
        self.situations = {k: v for k, v in self.situations.items() if now - v[1] <= 3}
        self.situations[key] = (record, now)
        while len(self.situations) > 512:
            self.situations.pop(next(iter(self.situations)))

    def add_task(self, task, kind):
        local = task["unity_only"]
        if not isinstance(local.get("decision_id"), str) or not local["decision_id"]:
            raise ValueError("Missing decision identity")
        if type(local.get("tick")) is not int or local["tick"] < 0:
            raise ValueError("Invalid sampling tick")
        if not all(isinstance(task.get(k), str) and task[k] for k in ("simId", "id", "code")):
            raise ValueError("Missing task identity")
        task_module = get_task(task["code"])
        key = (task["simId"], task["id"])
        if key in self.retired:
            raise ValueError("Retired Unity task")
        if self.current == key:
            if local["decision_id"] in self.seen:
                return False
            if local["tick"] < self.last_tick:
                raise ValueError("Older sampling tick")
        if kind == STOP:
            if key not in self.tasks:
                raise ValueError("Stop has no active task context")
            task = dict(task, params=self.tasks[key])
        params = task["params"]
        if type(params.get("platCode")) is not int or not 0 <= params["platCode"] <= 65535:
            raise ValueError("Invalid own platform")
        task_module.validate_params(params)
        # This radius table is also the exact entity manifest for this sample.
        radii = local["collision_radii_m"]
        if not isinstance(radii, dict) or not radii:
            raise ValueError("Missing snapshot entities/radii")
        expected = [int(pid) for pid in radii]
        if (any(str(int(pid)) != pid for pid in radii)
                or any(not 0 <= pid <= 65535 for pid in expected)
                or any(type(r) not in (int, float) or not math.isfinite(r) or r <= 0 for r in radii.values())
                or params["platCode"] not in expected):
            raise ValueError("Invalid snapshot entities/radii")
        task_module.validate_manifest(params, radii, expected)
        stamp = local["situation_timestamp"]
        if type(stamp) is not int or not 0 <= stamp <= 0xFFFFFFFF:
            raise ValueError("Invalid situation timestamp")
        if self.current != key:
            if self.current is not None:
                self.retired.add(self.current)
                self.tasks.pop(self.current, None)
            self.waiting.clear()
            self.current = key
            self.seen.clear()
            self.task_state = {}
            self.last_observations.clear()
        self.last_tick = local["tick"]
        self.seen.add(local["decision_id"])
        self.tasks[key] = params
        self.waiting[key] = (task, kind, time.monotonic())
        return True

    @staticmethod
    def identity(task):
        # Internal scheduler envelope only. No duplicate identities on the wire.
        return dict(session_id=task["simId"], episode_id=task["simId"], run_id=task["id"],
                    decision_id=task["unity_only"]["decision_id"], tick=task["unity_only"]["tick"])

    def finish(self, key):
        self.waiting.pop(key, None)
        self.tasks.pop(key, None)
        self.retired.add(key)
        self.last_observations.pop(key, None)

    def ready(self):
        result = []
        for key, (task, kind, created) in list(self.waiting.items()):
            local = task["unity_only"]
            stamp = local["situation_timestamp"]
            entities = [int(pid) for pid in local["collision_radii_m"]]
            now = time.monotonic()
            records = [self.situations.get((stamp, pid)) for pid in entities]
            records = [item[0] if item is not None and now - item[1] <= 3 else None for item in records]
            if all(r is not None for r in records):
                try:
                    packet = self.observation(dict(task, situation=records), kind)
                    self.last_observations[key] = packet['observation']
                    result.append((task, packet, None))
                except (ValueError, KeyError, TypeError, OverflowError) as error:
                    result.append((task, None, str(error)))
                del self.waiting[key]
                for pid in entities:
                    self.situations.pop((stamp, pid), None)
            elif time.monotonic() - created > 3:
                result.append((task, None, "Incomplete Unity situation snapshot; use Retry Jev"))
                del self.waiting[key]
        return result

    def observation(self, task, kind):
        local = task["unity_only"]
        params = task["params"]
        boat = next(r for r in task["situation"] if r["platformId"] == params["platCode"])
        if boat["targetSpeed"] is None:
            raise ValueError("Unknown own speed")
        boat = dict(boat, collision_radius_m=local["collision_radii_m"][str(boat["platformId"])])
        if abs(boat["targetSpeed"]) < 1e-6:
            boat["targetCourse"] = None
        task_module = get_task(task["code"])
        resolver = getattr(task_module, 'resolve_target_platform_id', None)
        target_id = resolver(params, task['situation']) if resolver else task_module.target_platform_id(params)
        maximum = local["max_speed_knots"]
        if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum <= 0:
            raise ValueError("Invalid candidate speed limit")
        obs = dict(boat=boat, obstacles=[],
                   planning=dict(action_duration_s=local["action_duration_s"], simulation_paused_while_deciding=True),
                   execution=local["execution"],
                   action_space=dict(speeds_knots=[maximum * f for f in SPEED_FRACTIONS],
                                     courses_deg=list(task_module.COURSES_DEG)), model_speed_unit=self.speed_unit)
        obs["operation"] = "stop" if kind == STOP else "execute"
        task_module.adapt_observation(params, local, task["situation"], obs, self.task_state)
        if kind == STOP:
            obs["task"]["stop_speed_mps"] = local["stop_speed_knots"] * KNOT_MPS
        for record in task["situation"]:
            if record["platformId"] != params["platCode"] and record["platformId"] != target_id:
                obs["obstacles"].append(dict(record, id=str(record["platformId"]),
                    radius_m=local["collision_radii_m"][str(record["platformId"])],
                    avoidance_radius_m=local["execution"]["obstacle_avoidance_radii_m"][str(record["platformId"])]))
        envelope = self.identity(task)
        return dict(envelope, type="stop_request" if kind == STOP else "decision_request", observation=obs)
