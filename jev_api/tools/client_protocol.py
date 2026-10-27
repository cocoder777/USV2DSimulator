"""甲方大端 UDP 帧；只处理编码、量化和分片，不参与决策。"""

import datetime
import json
import math
import struct
import time

TASK, CONTROL, STATUS, STOP, SITUATION = 0x0660, 0x0662, 0x0664, 0x0666, 0x0060
JSON_IDS = (TASK, STATUS, STOP)
HEADER = struct.Struct(">BBHIHH")
SITUATION_BODY = struct.Struct(">hiiHHHhhHBBI")
MAX_BODY = 1450


def timestamp(algorithm=True):
    tz = datetime.timezone.utc if algorithm else datetime.timezone(datetime.timedelta(hours=8))
    now = datetime.datetime.now(tz)
    return int((now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
               * (10000 if algorithm else 1000))


def header(kind, length, stamp, total=None, index=1):
    value = HEADER.pack(0, 0, length, stamp, 0, kind)
    return value if total is None else value + bytes((total, index))


def parse_header(data):
    if len(data) < 12:
        raise ValueError("Short protocol header")
    seq, unit, length, stamp, reserved, kind = HEADER.unpack_from(data)
    if length != len(data) or reserved != 0:
        raise ValueError("Invalid frame length/reserved")
    if kind not in (*JSON_IDS, CONTROL, SITUATION):
        raise ValueError("Unknown information unit")
    if kind in JSON_IDS:
        if seq or unit or len(data) < 14:
            raise ValueError("Invalid algorithm header")
        total, index = data[12:14]
        if not 1 <= index <= total <= 10 or len(data) - 14 > MAX_BODY:
            raise ValueError("Invalid fragment counters/size")
    else:
        total, index = 1, 1
    return kind, stamp, total, index


def json_packets(content, kind, stamp=None):
    if kind not in JSON_IDS:
        raise ValueError("Expected algorithm JSON unit")
    body = json.dumps(content, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if not body or len(body) > min(65535, MAX_BODY * 10 - 2):
        raise ValueError("JSON exceeds ten-fragment budget")
    stamp = timestamp() if stamp is None else stamp
    pieces = [body[:MAX_BODY - 2]]
    pieces.extend(body[i:i + MAX_BODY] for i in range(MAX_BODY - 2, len(body), MAX_BODY))
    return [header(kind, 14 + len(piece) + (2 if i == 1 else 0), stamp, len(pieces), i)
            + (struct.pack(">H", len(body)) if i == 1 else b"") + piece
            for i, piece in enumerate(pieces, 1)]


class Reassembler:
    def __init__(self, timeout=3.0):
        self.timeout = timeout
        self.pending = {}

    def add(self, data, source):
        kind, stamp, total, index = parse_header(data)
        if kind not in JSON_IDS:
            raise ValueError("Expected JSON frame")
        now = time.monotonic()
        self.pending = {k: v for k, v in self.pending.items() if now - v[0] <= self.timeout}
        key = (source, kind, stamp, total)
        if key not in self.pending:
            if len(self.pending) >= 128:
                self.pending.pop(next(iter(self.pending)))
            self.pending[key] = [now, None, {}]
        entry = self.pending[key]
        if index == 1:
            if len(data) < 16:
                raise ValueError("Missing contentLength")
            length = struct.unpack_from(">H", data, 14)[0]
            if not 0 < length <= MAX_BODY * 10 - 2 or entry[1] not in (None, length):
                raise ValueError("Invalid contentLength")
            entry[1], piece = length, data[16:]
        else:
            piece = data[14:]
        if index in entry[2] and entry[2][index] != piece:
            del self.pending[key]
            raise ValueError("Conflicting fragment")
        entry[2][index] = piece
        if entry[1] is None or len(entry[2]) != total:
            return None
        del self.pending[key]
        body = b"".join(entry[2][i] for i in range(1, total + 1))
        if len(body) != entry[1]:
            raise ValueError("Reassembled length mismatch")
        value = json.loads(body.decode("utf-8"), parse_constant=lambda v: _invalid(v))
        if not isinstance(value, dict):
            raise ValueError("Expected JSON object")
        return value


def _invalid(value):
    raise ValueError("Invalid JSON constant: " + value)


def decode_situation(data):
    kind, stamp, _, _ = parse_header(data)
    if kind != SITUATION or len(data) != 40:
        raise ValueError("Expected 40-byte situation")
    direction, lat, lon, pid, sign, course, height, speed, typ, confidence, threat, batch = SITUATION_BODY.unpack_from(data, 12)
    return dict(direction=direction, latitude=lat * 90 / 2**31, longitude=lon * 180 / 2**31,
                platformId=pid, sign=sign, targetCourse=None if course == 65535 else course,
                height=height, targetSpeed=None if speed == -1 else speed * .1, type=typ,
                targetTypeConfid=None if confidence == 255 else confidence,
                threatLevel=None if threat == 255 else threat, batch=batch, timestamp=stamp)


def control_packet(platform, speed, course, stamp=None):
    if not all(math.isfinite(v) for v in (speed, course)) or not 0 <= speed <= 655.35 or not 0 <= course < 360:
        raise ValueError("Invalid control")
    body = struct.pack(">IHH", platform, round(speed * 100), round(course * 100) % 36000)
    return header(CONTROL, 20, timestamp(False) if stamp is None else stamp) + body


class UnitySnapshotAdapter:
    """按采样标识汇齐组播态势，再适配 guoliang 风格的 task+situation 输入。"""
    def __init__(self, speed_unit="knots"):
        self.speed_unit = speed_unit
        self.situations = {}
        self.waiting = {}
        self.tasks = {}
        self.retired = set()
        self.current = None
        self.seen = set()
        self.last_tick = -1
        self.following_course = None
        self.following_stationary = False

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
        if task["code"] not in ("trackingMode", "targetFollowing"):
            raise ValueError("Expected trackingMode or targetFollowing")
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
        if task["code"] == "targetFollowing":
            lower, upper = params.get("minFollowingDistanceM"), params.get("maxFollowingDistanceM")
            if (type(params.get("targetPlatformId")) is not int or not 0 <= params["targetPlatformId"] <= 65535
                    or params["targetPlatformId"] == params["platCode"]
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in (lower, upper))
                    or not 0 < lower < upper):
                raise ValueError("Invalid target following task")
        elif (type(params.get("platCode")) is not int or not 0 <= params["platCode"] <= 65535
                or type(params.get("planWaypointNum")) is not int
                or not isinstance(params.get("planWaypointList"), list)
                or not params["planWaypointList"]
                or params["planWaypointNum"] != len(params["planWaypointList"])
                or type(params.get("currentRoute")) is not int
                or not 0 <= params["currentRoute"] < params["planWaypointNum"]):
            raise ValueError("Invalid trackingMode task or completed waypoint count")
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
        if task["code"] == "targetFollowing":
            target_id = params["targetPlatformId"]
            if target_id not in expected or params["minFollowingDistanceM"] <= radii[str(target_id)] + radii[str(params["platCode"])]:
                raise ValueError("Missing target or following minimum inside combined hulls")
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
            self.following_course = None
            self.following_stationary = False
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
        from tools.waypoint_geometry import KNOT_MPS
        import copy
        local = task["unity_only"]
        params = task["params"]
        boat = next(r for r in task["situation"] if r["platformId"] == params["platCode"])
        if boat["targetSpeed"] is None:
            raise ValueError("Unknown own speed")
        boat = dict(boat, collision_radius_m=local["collision_radii_m"][str(boat["platformId"])])
        if abs(boat["targetSpeed"]) < 1e-6:
            boat["targetCourse"] = None
        following = task["code"] == "targetFollowing"
        maximum = local["max_speed_knots"]
        if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum <= 0:
            raise ValueError("Invalid candidate speed limit")
        obs = dict(boat=boat, obstacles=[],
                   planning=dict(action_duration_s=local["action_duration_s"], simulation_paused_while_deciding=True),
                   execution=local["execution"],
                   action_space=dict(speeds_knots=[maximum * f for f in (0, .05, .1, .2, .35, .5, .75, 1)],
                                     courses_deg=list(range(0, 360, 45))), model_speed_unit=self.speed_unit)
        if following:
            from tools.waypoint_geometry import xy
            target = next(r for r in task["situation"] if r["platformId"] == params["targetPlatformId"])
            speed, course = target["targetSpeed"], target["targetCourse"]
            if speed is None or speed < 0 or (speed > 1e-5 and course is None):
                raise ValueError("Unknown or unsupported target motion")
            speed_mps = speed * KNOT_MPS
            self.following_stationary = speed_mps < (0.2 if self.following_stationary else 0.1)
            if not self.following_stationary and course is not None:
                self.following_course = course
            if self.following_course is None:
                delta = xy((target["longitude"], target["latitude"]), (boat["longitude"], boat["latitude"]))
                self.following_course = math.degrees(math.atan2(delta[0], delta[1])) % 360
            obs["target"] = dict(target, radius_m=local["collision_radii_m"][str(params["targetPlatformId"])])
            obs["task"] = dict(params, mode="target_following", reference_course_deg=self.following_course,
                               target_stationary=self.following_stationary)
        else:
            points = copy.deepcopy(params["planWaypointList"])
            for point in points:
                point["routeSpeed"] *= KNOT_MPS
            obs["task"] = dict(params, planWaypointList=points, routeSpeed_unit="m/s",
                               next_waypoint_index=params["currentRoute"], arrival_radius_m=local["arrival_radius_m"],
                               stop_speed_mps=local["stop_speed_knots"] * KNOT_MPS)
        if kind == STOP:
            obs["task"] = dict(obs["task"], mode="immediate_stop", stop_speed_mps=local["stop_speed_knots"] * KNOT_MPS)
        for record in task["situation"]:
            if record["platformId"] != params["platCode"] and not (following and record["platformId"] == params["targetPlatformId"]):
                obs["obstacles"].append(dict(record, id=str(record["platformId"]),
                    radius_m=local["collision_radii_m"][str(record["platformId"])],
                    avoidance_radius_m=local["execution"]["obstacle_avoidance_radii_m"][str(record["platformId"])]))
        envelope = self.identity(task)
        return dict(envelope, type="stop_request" if kind == STOP else "decision_request", observation=obs)
