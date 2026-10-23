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

    def add_situation(self, record):
        key = record["timestamp"], record["platformId"]
        self.situations[key] = record
        while len(self.situations) > 512:
            self.situations.pop(next(iter(self.situations)))

    def add_task(self, task, kind):
        local = task["unity_only"]
        for field in ("session_id", "episode_id", "run_id", "decision_id"):
            if not isinstance(local.get(field), str) or not local[field]:
                raise ValueError("Missing Unity identity")
        if type(local.get("tick")) is not int or local["tick"] < 0:
            raise ValueError("Invalid sampling tick")
        if not all(isinstance(task.get(k), str) and task[k] for k in ("simId", "id", "code")):
            raise ValueError("Missing task identity")
        if task["code"] != "trackingMode":
            raise ValueError("Expected trackingMode")
        expected = local["situation_platform_ids"]
        if not isinstance(expected, list) or not expected or len(set(expected)) != len(expected):
            raise ValueError("Invalid situation manifest")
        if any(type(pid) is not int or not 0 <= pid <= 65535 for pid in expected):
            raise ValueError("Invalid platform ID")
        own = task.get("params", {}).get("platCode", local.get("platCode"))
        if own not in expected:
            raise ValueError("Manifest has no own boat")
        key = (task["simId"], task["id"])
        self.waiting[key] = (task, kind, time.monotonic())
        while len(self.waiting) > 16:
            self.waiting.pop(next(iter(self.waiting)))

    def ready(self):
        result = []
        for key, (task, kind, created) in list(self.waiting.items()):
            local = task["unity_only"]
            stamp = local["situation_timestamp"]
            records = [self.situations.get((stamp, pid)) for pid in local["situation_platform_ids"]]
            if all(r is not None for r in records):
                try:
                    packet = self.observation(dict(task, situation=records), kind)
                    result.append((task, packet, None))
                except (ValueError, KeyError, TypeError, OverflowError) as error:
                    result.append((task, None, str(error)))
                del self.waiting[key]
            elif time.monotonic() - created > 3:
                result.append((task, None, "Incomplete Unity situation snapshot; use Retry Jev"))
                del self.waiting[key]
        return result

    def observation(self, task, kind):
        from tools.waypoint_geometry import KNOT_MPS
        import copy
        local = task["unity_only"]
        params = task.get("params", {"platCode": local.get("platCode"), "planWaypointList": []})
        boat = next(r for r in task["situation"] if r["platformId"] == params["platCode"])
        if boat["targetSpeed"] is None:
            raise ValueError("Unknown own speed")
        boat = dict(boat, collision_radius_m=local["collision_radii_m"][str(boat["platformId"])])
        if abs(boat["targetSpeed"]) < 1e-6:
            boat["targetCourse"] = None
        points = copy.deepcopy(params["planWaypointList"])
        for point in points:
            point["routeSpeed"] *= KNOT_MPS
        obs = dict(boat=boat, obstacles=[], planning=local["planning"], execution=local["execution"],
                   action_space=local["action_space"], model_speed_unit=self.speed_unit,
                   task=dict(params, planWaypointList=points, routeSpeed_unit="m/s",
                             next_waypoint_index=local["next_waypoint_index"],
                             arrival_radius_m=local["arrival_radius_m"],
                             stop_speed_mps=local["stop_speed_knots"] * KNOT_MPS))
        if kind == STOP:
            obs["task"] = dict(mode="immediate_stop", stop_speed_mps=local["stop_speed_knots"] * KNOT_MPS)
        for record in task["situation"]:
            if record["platformId"] != params["platCode"]:
                obs["obstacles"].append(dict(record, id=str(record["platformId"]),
                    radius_m=local["collision_radii_m"][str(record["platformId"])]))
        envelope = {k: local[k] for k in ("session_id", "episode_id", "run_id", "decision_id", "tick")}
        return dict(envelope, type="stop_request" if kind == STOP else "decision_request", observation=obs)
