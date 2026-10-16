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
    "Strong negative acceleration; rapidly brake forward motion, or build reverse speed.",
    "Medium negative acceleration; clearly reduce forward speed.",
    "Mild negative acceleration; gently reduce forward speed.",
    "No longitudinal acceleration input; coast with actuator lag and drag.",
    "Small positive acceleration; gently increase forward speed or compensate drag.",
    "Medium positive acceleration; build forward speed.",
    "Large positive acceleration; increase forward speed quickly.",
    "Maximum positive acceleration; strongest forward speed increase.",
)
TURN_LABELS = (
    "Strong clockwise/right angular acceleration; build negative yaw rate or brake positive yaw rate strongly.",
    "Medium clockwise/right angular acceleration; increase rightward rotation or reduce leftward rotation.",
    "Mild clockwise/right angular acceleration; small adjustment toward rightward rotation.",
    "No angular acceleration input; yaw rate continues under actuator lag and damping.",
    "Mild counterclockwise/left angular acceleration; small adjustment toward leftward rotation.",
    "Medium counterclockwise/left angular acceleration; increase leftward rotation or reduce rightward rotation.",
    "Strong counterclockwise/left angular acceleration; build positive yaw rate or brake negative yaw rate strongly.",
)


def build_payload(observation, model="jev-latest"):
    state = copy.deepcopy(observation)
    planning = state["planning"]
    fractions = state["actions"]
    dyn = state["dynamics"]
    criteria_a = [label + f" Input is {float(f) * dyn['max_acceleration']:.4g} m/s^2."
                  for label, f in zip(ACCEL_LABELS, fractions["acceleration_fractions"])]
    criteria_t = [label + f" Input is {float(f) * dyn['max_angular_acceleration_deg']:.4g} deg/s^2."
                  for label, f in zip(TURN_LABELS, fractions["turn_fractions"])]
    state["objective"] = ("Follow the reference path forward, visit mission targets in order, "
                          "maintain cruise speed, and stop inside the end radius. "
                          "Choose acceleration INPUTS, not velocity, position or heading. "
                          "Turning right can be needed to stop an existing leftward yaw rate.")
    state["control_guidance"] = (
        "Navigation supplies speed and yaw-rate TRACKING GOALS. Each axis has independent physical forecasts. "
        "Select the input with the SMALLEST tracking_error for that axis. "
        "tracking_error already includes current actuator inertia and damping. Smaller is better. "
        "Do not choose by the input sign alone: an opposite angular input can reduce excessive existing yaw rate. "
        "Do not choose an input merely because its absolute magnitude is small. Compare the predicted goal errors.")
    state["plan_instructions"] = (
        f"Plan {planning['action_count']} ordered paired actions in this single request. "
        f"Each action lasts {planning['action_duration_s']:.6g} simulation seconds. "
        "Each interval has one longitudinal acceleration question and one angular acceleration question. "
        "All answers are collected before simulation resumes; intervals execute consecutively in index order.")
    questions = {}
    forecasts_by_axis = state.pop("input_forecasts", None)
    if forecasts_by_axis:
        state["forecast_horizon_s"] = forecasts_by_axis["horizon_s"]
    for i in range(planning["action_count"]):
        begin = i * planning["action_duration_s"]
        end = begin + planning["action_duration_s"]
        context = (f"Action {i + 1} of {planning['action_count']}; duration {planning['action_duration_s']:.6g} s. "
                   f"Plan input for interval [{begin:.3f}, {end:.3f}) simulation seconds after resuming from this frozen state. "
                   "Account for prior intervals, inertia, damping and the mission goals. ")
        for name, criteria, instructions in (
            ("acceleration", criteria_a, "How much signed longitudinal acceleration input is appropriate?"),
            ("turn", criteria_t, "How much signed angular acceleration input is appropriate?"),
        ):
            if forecasts_by_axis:
                forecasts = forecasts_by_axis[name]
                criteria = [{"meaning": c, "physical_prediction": p} for c, p in zip(criteria, forecasts)]
                instructions = ("Which input has the SMALLEST physical_prediction.tracking_error on this axis? "
                                "Select the closest predicted tracking result; all inputs are allowed.")
            questions[f"{name}_{i}"] = {
                "type": "choice", "instructions": context + instructions,
                "criteria": {str(j): c for j, c in enumerate(criteria)},
            }
    return {"model": model, "state": state, "questions": questions}


def decode_actions(response, count):
    answers = response.get("answers")
    if not isinstance(answers, dict) or len(answers) != 2 * count:
        raise ValueError("Expected exactly two answers per action")
    actions = []
    for i in range(count):
        action = {}
        for name, maximum in (("acceleration", 7), ("turn", 6)):
            answer = answers.get(f"{name}_{i}")
            if not isinstance(answer, dict) or answer.get("type") != "choice":
                raise ValueError(f"Missing or wrong answer type for {name}_{i}")
            choice = answer.get("choice")
            if not isinstance(choice, str) or choice not in tuple(str(n) for n in range(maximum + 1)):
                raise ValueError("Unknown choice")
            action[name + "_index"] = int(choice)
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
