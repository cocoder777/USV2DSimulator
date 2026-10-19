"""Jev decision worker. Unity owns simulation and pauses while this process calls a local or cloud decision service."""
import argparse
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import socket
import time
import urllib.error
import urllib.request
from urllib.parse import urlsplit

from choice_control import ACTION_CHOICES, MODEL_STATE_FIELDS, build_payload, decode_action, make_instructions


MAX_PACKET = 8192
PROTOCOL_VERSION = 5
IDENTITY_FIELDS = ("session_id", "episode_id", "run_id", "decision_id", "tick")
BACKENDS = {
    "laya": ("http://127.0.0.1:8010/v1/systemone", "multilingual", "compact"),
    "kev": ("http://127.0.0.1:8009/v1/systemone", "kev-latest", "original"),
    "cloud": ("https://api.typesafe.ai/v1/systemone", "jev-latest", "original"),
}


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



def api_key_for_endpoint(endpoint):
    parsed = urlsplit(endpoint)
    if parsed.username or parsed.password or parsed.scheme not in ("http", "https"):
        raise ValueError("Endpoint must be an HTTP(S) URL without embedded credentials")
    if parsed.hostname in ("127.0.0.1", "localhost", "::1"):
        return None
    if parsed.scheme != "https":
        raise ValueError("Remote endpoints require HTTPS")
    key = os.environ.get("JEV_API_KEY", "").strip()
    if not key:
        raise ValueError("Set JEV_API_KEY before using a remote endpoint")
    return key


def call_api(payload, key, endpoint):
    # Loopback requests never carry a cloud credential, including direct callers.
    key = None if urlsplit(endpoint).hostname in ("127.0.0.1", "localhost", "::1") else key
    headers = {"Content-Type": "application/json"}
    if key:
        headers["Authorization"] = "Bearer " + key
    request = urllib.request.Request(endpoint, data=dumps(payload).encode("utf-8"), method="POST",
                                     headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=None) as response:  # Deliberately no decision timeout.
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"Decision service returned HTTP {error.code}") from None
    except urllib.error.URLError as error:
        raise RuntimeError(f"Decision service unavailable: {error.reason}") from None


def decide(packet, settings, key, log):
    started = time.monotonic()
    envelope = {k: packet[k] for k in IDENTITY_FIELDS}
    envelope["version"] = PROTOCOL_VERSION
    payload = None
    raw = None
    profile = getattr(settings, "prompt_profile", "original")
    try:
        payload = build_payload(packet["observation"], settings.model, profile)
        if getattr(settings, "backend", None) == "laya":
            payload.update(max_len=4096, head_max_len=3072)
        raw = call_api(payload, key, settings.endpoint)
        action = decode_action(raw)
        result = dict(envelope, type="decision_result", action=action,
                      action_duration_s=packet["observation"]["planning"]["action_duration_s"])
    except Exception as error:
        result = dict(envelope, type="decision_error", error=f"{type(error).__name__}: {error}")
    if log:
        log.write(dumps({"event": "decision", "elapsed_s": time.monotonic() - started,
                         "backend": getattr(settings, "backend", "custom"), "prompt_profile": profile,
                         "usage": raw.get("usage") if isinstance(raw, dict) else None,
                         "request": packet, "payload": payload, "raw_response": raw, "result": result}) + "\n")
        log.flush()
    print(dumps({"event": result["type"], "tick": packet["tick"], "action": result.get("action"),
                 "error": result.get("error"), "elapsed_s": round(time.monotonic() - started, 3)}), flush=True)
    return result


def serve(settings):
    key = api_key_for_endpoint(settings.endpoint)
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
            try:
                sock.bind((settings.listen_host, settings.listen_port))
            except OSError as error:
                raise OSError(f"Cannot bind UDP {settings.listen_host}:{settings.listen_port}: {error}") from None
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            sock.settimeout(0.05)
            print(f"Jev worker listening on {settings.listen_host}:{settings.listen_port}; Unity={endpoint}; "
                  f"backend={settings.backend}; model={settings.model}; service={settings.endpoint}; "
                  f"prompt={settings.prompt_profile}", flush=True)
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
        pool.shutdown(wait=True)
        if log:
            log.close()


def parse_settings(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=5006)
    parser.add_argument("--backend", choices=BACKENDS, default="laya")
    parser.add_argument("--model")
    parser.add_argument("--endpoint")
    parser.add_argument("--prompt-profile", choices=("original", "compact"))
    parser.add_argument("--log")
    parser.add_argument("--exit-on-complete", action="store_true")
    settings = parser.parse_args(args)
    endpoint, model, profile = BACKENDS[settings.backend]
    settings.endpoint = settings.endpoint or endpoint
    settings.model = settings.model or model
    settings.prompt_profile = settings.prompt_profile or profile
    if any(not 1 <= p <= 65535 for p in (settings.port, settings.listen_port)):
        parser.error("Ports must be 1..65535")
    return settings


def main():
    settings = parse_settings()
    try:
        serve(settings)
    except KeyboardInterrupt:
        print("Worker interrupted. Unity remains paused at its next decision boundary.")
    except (OSError, ValueError) as error:
        raise SystemExit(f"Jev worker error: {error}") from None


if __name__ == "__main__":
    main()
