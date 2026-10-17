"""Jev decision worker. Unity owns simulation and pauses while this process calls HTTPS."""
import argparse
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import socket
import time
import urllib.error
import urllib.request

API_KEY = 'apikey_262fad86dabbdf64bad8847eff8ed10446c_2a28eda5476e1798a4d3c7c843d1c76302706522686efa24d124cf33938ed141'

MAX_PACKET = 8192
IDENTITY_FIELDS = ("session_id", "episode_id", "run_id", "decision_id", "tick")


def identity(packet):
    return tuple(packet.get(key) for key in IDENTITY_FIELDS)


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def valid_request(packet):
    if not isinstance(packet, dict) or packet.get("version") != 4 or packet.get("type") != "decision_request":
        return False
    if any(not isinstance(packet.get(k), str) or not packet[k] for k in IDENTITY_FIELDS[:-1]):
        return False
    if type(packet.get("tick")) is not int or packet["tick"] < 0:
        return False
    try:
        obs = packet["observation"]
        planning = obs["planning"]
        count, duration = planning["action_count"], planning["action_duration_s"]
        if type(count) is not int or not 1 <= count <= 10:
            return False
        if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
            return False
        return len(obs["actions"]["acceleration_fractions"]) == 8 and len(obs["actions"]["turn_fractions"]) == 7
    except (KeyError, TypeError):
        return False



ACCEL_LABELS = (
    "Maximum negative input; strongest reduction of signed speed.",
    "Medium negative input; clear reduction of signed speed.",
    "Mild negative input; gentle reduction of signed speed.",
    "Zero input; speed changes under actuator lag and drag.",
    "Small positive input; gentle increase of signed speed.",
    "Medium positive input; clear increase of signed speed.",
    "Large positive input; rapid increase of signed speed.",
    "Maximum positive input; strongest increase of signed speed.",
)
TURN_LABELS = (
    "Maximum clockwise input; strongly decrease signed yaw rate.",
    "Medium clockwise input; clearly decrease signed yaw rate.",
    "Mild clockwise input; gently decrease signed yaw rate.",
    "Zero input; rotation changes under actuator lag and damping.",
    "Mild counterclockwise input; gently increase signed yaw rate.",
    "Medium counterclockwise input; clearly increase signed yaw rate.",
    "Maximum counterclockwise input; strongly increase signed yaw rate.",
)
MODEL_STATE_FIELDS = ("coordinates", "planning", "boat", "target", "dynamics", "motion")


def make_instructions(axis, index, count, duration):
    command = {"acceleration": "longitudinal acceleration", "turn": "angular acceleration"}[axis]
    if count == 1:
        timing = (f"for the next {duration:.6g} seconds? "
                  "The supplied boat state is the state at the start of this interval.")
    else:
        begin, end = index * duration, (index + 1) * duration
        timing = (f"for action {index + 1} of {count}, during [{begin:.6g}, {end:.6g}) seconds "
                  "after simulation resumes? The supplied boat state is measured at time zero. "
                  "No updated state is supplied for later intervals. All actions are requested together "
                  "and executed in order without intermediate feedback.")
    return f"What signed {command} command is appropriate to reach the current target " + timing


def build_payload(observation, model="jev-latest"):
    state = {field: copy.deepcopy(observation[field]) for field in MODEL_STATE_FIELDS}
    planning, dyn = state["planning"], state["dynamics"]
    fractions = observation["actions"]
    criteria_a = [label + f" Command: {float(f) * dyn['max_acceleration']:.4g} m/s^2."
                  for label, f in zip(ACCEL_LABELS, fractions["acceleration_fractions"])]
    criteria_t = [label + f" Command: {float(f) * dyn['max_angular_acceleration_deg']:.4g} deg/s^2."
                  for label, f in zip(TURN_LABELS, fractions["turn_fractions"])]
    state["task"] = (
        "Navigate the boat to the current target using global XY coordinates. "
        "For a pass-through target, enter its arrival radius. "
        "For a stop target, be inside its arrival radius with absolute speed at or below the stop threshold. "
        "The target.requires_stop field specifies which rule applies. "
        "Commands are signed longitudinal acceleration and angular acceleration. "
        "Positive longitudinal acceleration increases signed speed; positive angular acceleration acts counterclockwise. "
        "Zero command does not immediately stop motion. "
        "Account for current velocity, rotation, actuator lag and damping.")
    questions = {}
    for i in range(planning["action_count"]):
        for axis, criteria in (("acceleration", criteria_a), ("turn", criteria_t)):
            questions[f"{axis}_{i}"] = {
                "type": "score",
                "instructions": make_instructions(axis, i, planning["action_count"], planning["action_duration_s"]),
                "criteria": criteria,
            }
    return {"model": model, "state": state, "questions": questions}


def decode_actions(response, count):
    answers = response.get("answers")
    expected = {f"{axis}_{i}" for i in range(count) for axis in ("acceleration", "turn")}
    if not isinstance(answers, dict) or set(answers) != expected:
        raise ValueError("Expected exactly two matching answers per action")
    actions = []
    for i in range(count):
        action = {}
        for name, maximum in (("acceleration", 7), ("turn", 6)):
            answer = answers[f"{name}_{i}"]
            if not isinstance(answer, dict) or answer.get("type") != "score":
                raise ValueError(f"Missing or wrong answer type for {name}_{i}")
            score = answer.get("score")
            if type(score) not in (int, float) or not math.isfinite(score) or not 0 <= score <= maximum:
                raise ValueError(f"Invalid score for {name}_{i}")
            # Nearest level, with exact half-levels rounded upward; keep Unity's discrete wire contract.
            action[name + "_index"] = math.floor(score + 0.5)
        actions.append(action)
    return actions


def call_api(payload, key, endpoint):
    request = urllib.request.Request(endpoint, data=dumps(payload).encode("utf-8"), method="POST",
                                     headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=None) as response:  # Deliberately no decision timeout.
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"Jev returned HTTP {error.code}") from None


def decide(packet, settings, key, log):
    started = time.monotonic()
    envelope = {k: packet[k] for k in IDENTITY_FIELDS}
    envelope["version"] = 4
    payload = None
    try:
        payload = build_payload(packet["observation"], settings.model)
        raw = call_api(payload, key, settings.endpoint)
        actions = decode_actions(raw, packet["observation"]["planning"]["action_count"])
        result = dict(envelope, type="decision_result", actions=actions,
                      action_duration_s=packet["observation"]["planning"]["action_duration_s"])
    except Exception as error:
        raw = None
        result = dict(envelope, type="decision_error", error=f"{type(error).__name__}: {error}")
    if log:
        log.write(dumps({"event": "decision", "elapsed_s": time.monotonic() - started,
                         "request": packet, "payload": payload, "raw_response": raw, "result": result}) + "\n")
        log.flush()
    print(dumps({"event": result["type"], "tick": packet["tick"], "actions": result.get("actions"),
                 "error": result.get("error"), "elapsed_s": round(time.monotonic() - started, 3)}), flush=True)
    return result


def serve(settings):
    key = API_KEY
    endpoint = (socket.gethostbyname(settings.host), settings.port)
    log = open(settings.log, "a", encoding="utf-8") if settings.log else None
    pool = ThreadPoolExecutor(max_workers=1)
    future = None
    active_request = None
    queued = None
    cache = OrderedDict()
    awaiting_ack = {}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((settings.listen_host, settings.listen_port))
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            sock.settimeout(0.05)
            print(f"Jev worker listening on {settings.listen_host}:{settings.listen_port}; Unity={endpoint}", flush=True)
            while True:
                if future is not None and future.done():
                    result = future.result()
                    request_id = identity(active_request)
                    cache[request_id] = result
                    if len(cache) > 256:
                        cache.popitem(last=False)
                    awaiting_ack[request_id] = (result, 0)
                    future = None
                    active_request = None
                if future is None and queued is not None:
                    active_request, queued = queued, None
                    future = pool.submit(decide, active_request, settings, key, log)
                now = time.monotonic()
                for request_id, (result, sent) in list(awaiting_ack.items()):
                    if now - sent >= 0.3:
                        sock.sendto(dumps(result).encode(), endpoint)
                        awaiting_ack[request_id] = (result, now)
                try:
                    data, source = sock.recvfrom(MAX_PACKET + 1)
                except socket.timeout:
                    continue
                except OSError as error:
                    if getattr(error, "winerror", None) in (10040, 10054):
                        continue
                    raise
                if source != endpoint or len(data) > MAX_PACKET:
                    continue
                try:
                    packet = json.loads(data.decode("utf-8"), parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(packet, dict):
                    continue
                if packet.get("type") == "state":
                    if (packet.get("mission") or {}).get("status") == "Completed" and settings.exit_on_complete:
                        if log:
                            log.write(dumps({"event": "completed", "state": packet}) + "\n"); log.flush()
                        print(dumps({"event": "completed", "simulation_time": packet.get("simulation_time"),
                                     "position": [packet.get("x"), packet.get("y")], "speed": packet.get("speed")}), flush=True)
                        return
                    continue
                request_id = identity(packet)
                if packet.get("type") == "decision_ack" and packet.get("version") == 4:
                    awaiting_ack.pop(request_id, None)
                    continue
                if not valid_request(packet):
                    continue
                # A new request supersedes unacknowledged results from earlier decisions.
                awaiting_ack = {k: v for k, v in awaiting_ack.items() if k == request_id}
                if request_id in cache:
                    sock.sendto(dumps(cache[request_id]).encode(), endpoint)
                elif future is not None and request_id == identity(active_request):
                    continue
                else:
                    queued = packet
    finally:
        pool.shutdown(wait=True)
        if log:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=5006)
    parser.add_argument("--model", default="jev-latest")
    parser.add_argument("--endpoint", default="https://api.typesafe.ai/v1/systemone")
    parser.add_argument("--log")
    parser.add_argument("--exit-on-complete", action="store_true")
    settings = parser.parse_args()
    if any(not 1 <= p <= 65535 for p in (settings.port, settings.listen_port)):
        parser.error("Ports must be 1..65535")
    try:
        serve(settings)
    except KeyboardInterrupt:
        print("Worker interrupted. Unity remains paused at its next decision boundary.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Jev worker error: {error}\n")


if __name__ == "__main__":
    main()
