"""Jev decision worker. Unity owns simulation and pauses while this process calls HTTPS."""
import argparse
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import json
import math
import socket
import time
import threading

from http_transport import JevTransport

from choice_control import ACTION_CHOICES, MODEL_STATE_FIELDS, build_payload, decode_action, make_instructions

API_KEY = 'apikey_262fad86dabbdf64bad8847eff8ed10446c_2a28eda5476e1798a4d3c7c843d1c76302706522686efa24d124cf33938ed141'

MAX_PACKET = 8192
PROTOCOL_VERSION = 5
IDENTITY_FIELDS = ("session_id", "episode_id", "run_id", "decision_id", "tick")


def identity(packet):
    return tuple(packet.get(key) for key in IDENTITY_FIELDS)


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def valid_request(packet):
    if not isinstance(packet, dict) or type(packet.get("version")) is not int or packet.get("version") != PROTOCOL_VERSION or packet.get("type") != "decision_request":
        return False
    if any(not isinstance(packet.get(k), str) or not packet[k] for k in IDENTITY_FIELDS[:-1]):
        return False
    if type(packet.get("tick")) is not int or packet["tick"] < 0:
        return False
    try:
        obs = packet["observation"]
        planning = obs["planning"]
        duration = planning["action_duration_s"]
        if "action_count" in planning or "actions" in obs:
            return False
        if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
            return False
        return len(obs["action_space"]["acceleration_fractions"]) == 8 and len(obs["action_space"]["turn_fractions"]) == 7
    except (KeyError, TypeError):
        return False



_worker = threading.local()


def call_api(payload, key, endpoint, timings=None, transport="persistent"):
    if not hasattr(_worker, "http"):
        _worker.http = JevTransport()
    return _worker.http.call(payload, key, endpoint, timings, transport)


def close_api():
    if hasattr(_worker, "http"):
        _worker.http.close()


def decide(packet, settings, key, log):
    started = time.monotonic()
    envelope = {k: packet[k] for k in IDENTITY_FIELDS}
    envelope["version"] = PROTOCOL_VERSION
    payload = None
    timings = {}
    try:
        payload = build_payload(packet["observation"], settings.model)
        timings["build_payload_s"] = time.monotonic() - started
        raw = call_api(payload, key, settings.endpoint, timings,
                       getattr(settings, "http_transport", "persistent"))
        decode_started = time.monotonic()
        action = decode_action(raw)
        timings["decode_action_s"] = time.monotonic() - decode_started
        result = dict(envelope, type="decision_result", action=action,
                      action_duration_s=packet["observation"]["planning"]["action_duration_s"])
    except Exception as error:
        raw = None
        result = dict(envelope, type="decision_error", error=f"{type(error).__name__}: {error}")
    if log:
        log.write(dumps({"event": "decision", "elapsed_s": time.monotonic() - started,
                         "timings": timings, "request": packet, "payload": payload, "raw_response": raw, "result": result}) + "\n")
        log.flush()
    print(dumps({"event": result["type"], "tick": packet["tick"], "action": result.get("action"),
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
            sock.settimeout(0.01)
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
                    if packet.get("version") != PROTOCOL_VERSION:
                        continue
                    if (packet.get("mission") or {}).get("status") == "Completed" and settings.exit_on_complete:
                        if log:
                            log.write(dumps({"event": "completed", "state": packet}) + "\n"); log.flush()
                        print(dumps({"event": "completed", "simulation_time": packet.get("simulation_time"),
                                     "position": [packet.get("x"), packet.get("y")], "speed": packet.get("speed")}), flush=True)
                        return
                    continue
                request_id = identity(packet)
                if packet.get("type") == "decision_ack" and packet.get("version") == PROTOCOL_VERSION:
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
        pool.submit(close_api).result()
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
    parser.add_argument("--http-transport", choices=("persistent", "urllib"), default="persistent",
                        help="Reuse HTTPS connections (default), or use the original urllib transport")
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
