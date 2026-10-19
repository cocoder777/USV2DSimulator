"""Deterministic controller worker for the existing 56-action UDP protocol.

It never changes Unity physics or the mission judge. It only chooses one of the
same acceleration/turn index pairs a model would choose.
"""
import argparse
from collections import OrderedDict
import json
import math
import socket
import time
from dataclasses import dataclass

MAX_PACKET = 8192
PROTOCOL_VERSION = 5
IDENTITY_FIELDS = ("session_id", "episode_id", "run_id", "decision_id", "tick")


@dataclass(frozen=True)
class ControllerConfig:
    fixed_dt: float = 0.02
    heading_gain: float = 1.5
    yaw_gain: float = 4.0
    speed_gain: float = 1.5
    cruise_speed: float = 2.5
    creep_speed: float = 0.4
    deceleration: float = 1.0
    stop_margin: float = 0.2
    heading_deadband_deg: float = 1.0
    yaw_deadband_deg_s: float = 1.0
    bearing_weight: float = 0.15
    yaw_weight: float = 0.002
    stop_speed_weight: float = 2.0
    reverse_weight: float = 0.5
    switch_penalty: float = 0.01


@dataclass(frozen=True)
class PredictedState:
    x: float
    y: float
    heading_deg: float
    speed_mps: float
    yaw_rate_deg_s: float


def wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def target_frame(observation):
    boat = observation["boat"]
    target = observation["target"]
    bx, by = boat["position_xy_m"]
    tx, ty = target["position_xy_m"]
    heading = math.radians(boat["heading_deg"])
    dx, dy = tx - bx, ty - by
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    left = -dx * math.sin(heading) + dy * math.cos(heading)
    return forward, left, math.hypot(dx, dy)


def target_bearing_deg(observation):
    forward, left, _ = target_frame(observation)
    return math.degrees(math.atan2(left, forward))


def action_values(observation, acceleration_index, turn_index):
    fractions = observation["action_space"]
    dynamics = observation["dynamics"]
    linear = fractions["acceleration_fractions"][acceleration_index] * dynamics["max_acceleration"]
    angular = fractions["turn_fractions"][turn_index] * dynamics["max_angular_acceleration_deg"]
    return linear, angular


def prediction_steps(observation, config):
    duration = observation["planning"]["action_duration_s"]
    ratio = duration / config.fixed_dt
    count = int(round(ratio))
    if count < 1 or abs(count - ratio) > 0.0001:
        raise ValueError("Action duration must contain whole fixed steps")
    return count


def predict_action(observation, acceleration_index, turn_index, config=None):
    config = config or ControllerConfig()
    boat = observation["boat"]
    dynamics = observation["dynamics"]
    linear, angular = action_values(observation, acceleration_index, turn_index)
    x, y = boat["position_xy_m"]
    heading = math.radians(boat["heading_deg"])
    speed = boat["speed_mps"]
    yaw = boat["yaw_rate_deg_s"]
    for _ in range(prediction_steps(observation, config)):
        speed = min(dynamics["max_forward_speed"], max(-dynamics["max_reverse_speed"],
            speed + (linear - dynamics["linear_damping"] * speed) * config.fixed_dt))
        yaw = min(dynamics["max_yaw_rate_deg"], max(-dynamics["max_yaw_rate_deg"],
            yaw + (angular - dynamics["angular_damping"] * yaw) * config.fixed_dt))
        heading += math.radians(yaw) * config.fixed_dt
        x += speed * math.cos(heading) * config.fixed_dt
        y += speed * math.sin(heading) * config.fixed_dt
    return PredictedState(x, y, math.degrees(heading), speed, yaw)


def nearest_index(fractions, maximum, desired):
    values = [fraction * maximum for fraction in fractions]
    return min(range(len(values)), key=lambda index: (abs(values[index] - desired), index))


def nearest_action(observation, linear, angular):
    action_space = observation["action_space"]
    dynamics = observation["dynamics"]
    return {
        "acceleration_index": nearest_index(action_space["acceleration_fractions"],
                                            dynamics["max_acceleration"], linear),
        "turn_index": nearest_index(action_space["turn_fractions"],
                                    dynamics["max_angular_acceleration_deg"], angular),
    }


def _clamp(value, limit):
    return min(limit, max(-limit, value))


def _desired_speed(observation, distance, bearing, config):
    target = observation["target"]
    if not target.get("requires_stop"):
        alignment = max(0.0, math.cos(math.radians(bearing)))
        return config.cruise_speed * alignment
    radius = target["arrival_radius_m"]
    if distance <= radius:
        return 0.0
    braking = math.sqrt(2.0 * config.deceleration * max(0.0, distance - radius - config.stop_margin))
    return min(config.cruise_speed, max(config.creep_speed, braking))


def pd_action(observation, config=None, previous_action=None):
    config = config or ControllerConfig()
    boat = observation["boat"]
    dynamics = observation["dynamics"]
    bearing = target_bearing_deg(observation)
    _, _, distance = target_frame(observation)
    desired_speed = _desired_speed(observation, distance, bearing, config)
    if abs(bearing) > 120.0 and boat["speed_mps"] > 0.5:
        desired_speed = 0.0
    linear = (_clamp(dynamics["linear_damping"] * boat["speed_mps"]
                     + config.speed_gain * (desired_speed - boat["speed_mps"]),
                     dynamics["max_acceleration"]))
    if abs(bearing) <= config.heading_deadband_deg and abs(boat["yaw_rate_deg_s"]) <= config.yaw_deadband_deg_s:
        angular = 0.0
    else:
        desired_yaw = _clamp(config.heading_gain * bearing, dynamics["max_yaw_rate_deg"])
        angular = _clamp(dynamics["angular_damping"] * desired_yaw
                         + config.yaw_gain * (desired_yaw - boat["yaw_rate_deg_s"]),
                         dynamics["max_angular_acceleration_deg"])
    return nearest_action(observation, linear, angular)


def _allowed_speed(observation, distance, config):
    target = observation["target"]
    if not target.get("requires_stop"):
        return None
    radius = target["arrival_radius_m"]
    if distance <= radius:
        return 0.0
    return max(config.creep_speed,
               math.sqrt(2.0 * config.deceleration * max(0.0, distance - radius - config.stop_margin)))


def _action_cost(observation, predicted, config, previous_action, acceleration_index, turn_index):
    target = observation["target"]
    tx, ty = target["position_xy_m"]
    dx, dy = tx - predicted.x, ty - predicted.y
    distance = math.hypot(dx, dy)
    boat_heading = math.radians(predicted.heading_deg)
    forward = dx * math.cos(boat_heading) + dy * math.sin(boat_heading)
    left = -dx * math.sin(boat_heading) + dy * math.cos(boat_heading)
    bearing = math.degrees(math.atan2(left, forward))
    cost = distance
    cost += config.bearing_weight * abs(bearing)
    cost += config.yaw_weight * abs(predicted.yaw_rate_deg_s)
    allowed = _allowed_speed(observation, distance, config)
    if allowed is not None:
        cost += config.stop_speed_weight * max(0.0, abs(predicted.speed_mps) - allowed)
    else:
        cost += config.reverse_weight * max(0.0, -predicted.speed_mps)
    if previous_action is not None:
        cost += config.switch_penalty * (
            (previous_action["acceleration_index"] != acceleration_index)
            + (previous_action["turn_index"] != turn_index))
    return cost


def lookahead_action(observation, config=None, previous_action=None):
    config = config or ControllerConfig()
    action_space = observation["action_space"]
    best = None
    for acceleration_index in range(len(action_space["acceleration_fractions"])):
        for turn_index in range(len(action_space["turn_fractions"])):
            predicted = predict_action(observation, acceleration_index, turn_index, config)
            linear, angular = action_values(observation, acceleration_index, turn_index)
            cost = _action_cost(observation, predicted, config, previous_action,
                                acceleration_index, turn_index)
            key = (round(cost, 12), abs(linear), abs(angular), acceleration_index, turn_index)
            if best is None or key < best[0]:
                best = (key, {"acceleration_index": acceleration_index, "turn_index": turn_index})
    return best[1]


def choose_action(observation, method="lookahead", config=None, previous_action=None):
    if method == "pd":
        return pd_action(observation, config, previous_action)
    if method == "lookahead":
        return lookahead_action(observation, config, previous_action)
    raise ValueError("Unknown controller method: " + method)


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def identity(packet):
    return tuple(packet.get(key) for key in IDENTITY_FIELDS)


def valid_request(packet):
    if not isinstance(packet, dict) or type(packet.get("version")) is not int or packet.get("version") != PROTOCOL_VERSION or packet.get("type") != "decision_request":
        return False
    if any(not isinstance(packet.get(key), str) or not packet[key] for key in IDENTITY_FIELDS[:-1]):
        return False
    if type(packet.get("tick")) is not int or packet["tick"] < 0:
        return False
    try:
        observation = packet["observation"]
        planning = observation["planning"]
        duration = planning["action_duration_s"]
        if "action_count" in planning or "actions" in observation:
            return False
        if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
            return False
        return (len(observation["action_space"]["acceleration_fractions"]) == 8
                and len(observation["action_space"]["turn_fractions"]) == 7)
    except (KeyError, TypeError):
        return False


def _envelope(packet):
    result = {key: packet[key] for key in IDENTITY_FIELDS}
    result["version"] = PROTOCOL_VERSION
    return result


def decision_result(packet, method="lookahead", config=None, previous_action=None):
    action = choose_action(packet["observation"], method, config, previous_action)
    result = _envelope(packet)
    result.update(type="decision_result", action=action,
                  action_duration_s=packet["observation"]["planning"]["action_duration_s"])
    return result


def decision_error(packet, error):
    result = _envelope(packet)
    result.update(type="decision_error", error=f"{type(error).__name__}: {error}")
    return result


def serve(settings):
    config = ControllerConfig(fixed_dt=settings.fixed_dt)
    endpoint = (socket.gethostbyname(settings.host), settings.port)
    log = open(settings.log, "a", encoding="utf-8") if settings.log else None
    cache = OrderedDict()
    awaiting = {}
    previous_action = None
    last_run = None
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
            print(f"Classic controller listening on {settings.listen_host}:{settings.listen_port}; "
                  f"Unity={endpoint}; method={settings.method}; fixed_dt={settings.fixed_dt}", flush=True)
            while True:
                now = time.monotonic()
                for request_id, (result, sent) in list(awaiting.items()):
                    if now - sent >= 0.3:
                        sock.sendto(dumps(result).encode(), endpoint)
                        awaiting[request_id] = (result, now)
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
                    packet = json.loads(data.decode("utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(packet, dict):
                    continue
                if packet.get("type") == "state":
                    if packet.get("version") != PROTOCOL_VERSION:
                        continue
                    if settings.exit_on_complete and (packet.get("mission") or {}).get("status") == "Completed":
                        return
                    continue
                request_id = identity(packet)
                if packet.get("type") == "decision_ack" and packet.get("version") == PROTOCOL_VERSION:
                    awaiting.pop(request_id, None)
                    continue
                if not valid_request(packet):
                    continue
                run_key = (packet["session_id"], packet["episode_id"], packet["run_id"])
                if run_key != last_run:
                    previous_action = None
                    last_run = run_key
                if request_id in cache:
                    result = cache[request_id]
                else:
                    started = time.monotonic()
                    try:
                        result = decision_result(packet, settings.method, config, previous_action)
                        previous_action = result["action"]
                    except Exception as error:
                        result = decision_error(packet, error)
                    cache[request_id] = result
                    if len(cache) > 256:
                        cache.popitem(last=False)
                    if log:
                        log.write(dumps({"event": "decision", "elapsed_s": time.monotonic() - started,
                                         "method": settings.method, "request": packet, "result": result}) + "\n")
                        log.flush()
                    print(dumps({"event": result["type"], "tick": packet["tick"],
                                 "action": result.get("action"), "error": result.get("error"),
                                 "elapsed_s": round(time.monotonic() - started, 3)}), flush=True)
                awaiting = {request_id: (result, now)}
                sock.sendto(dumps(result).encode(), endpoint)
    finally:
        if log:
            log.close()


def parse_settings(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15005)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=15006)
    parser.add_argument("--method", choices=("pd", "lookahead"), default="lookahead")
    parser.add_argument("--fixed-dt", type=float, default=0.02)
    parser.add_argument("--log")
    parser.add_argument("--exit-on-complete", action="store_true")
    settings = parser.parse_args(args)
    if any(not 1 <= port <= 65535 for port in (settings.port, settings.listen_port)):
        parser.error("Ports must be 1..65535")
    if not math.isfinite(settings.fixed_dt) or settings.fixed_dt <= 0:
        parser.error("fixed_dt must be positive")
    return settings


def main():
    settings = parse_settings()
    try:
        serve(settings)
    except KeyboardInterrupt:
        print("Classic controller interrupted. Unity remains paused at its next decision boundary.")
    except (OSError, ValueError) as error:
        raise SystemExit(f"Classic controller error: {error}") from None


if __name__ == "__main__":
    main()
