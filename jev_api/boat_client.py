"""Unity 的 UDP 入口；默认暂停决策，HTTPS 在后台线程执行。"""

import argparse
import json
import math
import socket
import select
from tools.client_protocol import (TASK, CONTROL, STATUS, STOP, SITUATION, Reassembler,
    UnitySnapshotAdapter, parse_header, decode_situation, json_packets, control_packet, timestamp)

from decision_worker import DecisionScheduler, IDENTITY_FIELDS
from tools.waypoint_geometry import WaypointGeometry

API_KEY = "apikey_26639a7022497874a548d3a318ee22c06e7_61d7bc21b12e449581f1a9f4401e581a37cfde43e6f688f1ad383ff8ff815631"


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def valid_request(packet):
    if (
        not isinstance(packet, dict)
        or packet.get("type") not in ("decision_request", "stop_request")
    ):
        return False
    if any(
        not isinstance(packet.get(k), str) or not packet[k]
        for k in IDENTITY_FIELDS[:-1]
    ):
        return False
    if type(packet.get("tick")) is not int or packet["tick"] < 0:
        return False
    try:
        obs = packet["observation"]
        planning = obs["planning"]
        duration = planning["action_duration_s"]
        if "action_count" in planning or "actions" in obs:
            return False
        if (
            type(duration) not in (int, float)
            or not math.isfinite(duration)
            or duration <= 0
        ):
            return False
        boat, task, execution, space = (
            obs["boat"],
            obs["task"],
            obs["execution"],
            obs["action_space"],
        )

        def finite(v):
            return type(v) in (int, float) and math.isfinite(v)

        if not all(finite(boat[k]) for k in ("longitude", "latitude", "targetSpeed")):
            return False
        if not -180 <= boat["longitude"] <= 180 or not -90 <= boat["latitude"] <= 90:
            return False
        hull = boat.get("collision_radius_m", 0.5)
        if not finite(hull) or hull <= 0:
            return False
        obstacles = obs.get("obstacles", [])
        if not isinstance(obstacles, list):
            return False
        ids = set()
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                return False
            identifier = obstacle.get("id")
            if not isinstance(identifier, str) or not identifier or identifier in ids:
                return False
            ids.add(identifier)
            obstacle_speed, obstacle_course = obstacle.get("targetSpeed"), obstacle.get("targetCourse")
            if obstacle_speed is not None and not finite(obstacle_speed):
                return False
            if obstacle_course is not None and (not finite(obstacle_course) or not 0 <= obstacle_course < 360):
                return False
            if (not all(finite(obstacle[k]) for k in ("longitude", "latitude", "radius_m"))
                or not -180 <= obstacle["longitude"] <= 180
                or not -90 <= obstacle["latitude"] <= 90 or obstacle["radius_m"] <= 0):
                return False
        course = boat["targetCourse"]
        if course is not None and (not finite(course) or not 0 <= course < 360):
            return False
        if course is None and abs(boat["targetSpeed"]) > 1e-5:
            return False
        if not all(
            finite(execution[k]) and execution[k] >= 0
            for k in ("speed_transition_time_s", "course_transition_time_s")
        ):
            return False
        speeds, courses = space["speeds_knots"], space["courses_deg"]
        if (
            len(speeds) != 8
            or len(courses) != 8
            or not all(finite(v) and v >= 0 for v in speeds)
            or not all(finite(v) and 0 <= v < 360 for v in courses)
        ):
            return False
        if packet["type"] == "stop_request":
            return (
                task.get("mode") == "immediate_stop"
                and finite(task["stop_speed_mps"])
                and task["stop_speed_mps"] >= 0
            )
        if task.get("mode") == "immediate_stop":
            return False
        if type(task["planWaypointNum"]) is not int or task["planWaypointNum"] != len(
            task["planWaypointList"]
        ):
            return False
        if (
            type(task["next_waypoint_index"]) is not int
            or not 0 <= task["next_waypoint_index"] < task["planWaypointNum"]
        ):
            return False
        if (
            task["routeSpeed_unit"] != "m/s"
            or not finite(task["arrival_radius_m"])
            or task["arrival_radius_m"] <= 0
            or not finite(task["stop_speed_mps"])
            or task["stop_speed_mps"] < 0
        ):
            return False
        for p in task["planWaypointList"]:
            if (
                not all(finite(p[k]) for k in ("routeLon", "routeLat", "routeSpeed"))
                or p["routeSpeed"] < 0
                or not -180 <= p["routeLon"] <= 180
                or not -90 <= p["routeLat"] <= 90
            ):
                return False
        WaypointGeometry(obs)
        return True
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def report_decision(record, log):
    """完整记录写文件，终端只显示便于比较的耗时和连接状态。"""
    if log:
        log.write(dumps(record) + "\n")
        log.flush()
    result, timings = record["result"], record["timings"]
    connect_s = sum(
        timings.get(field, 0)
        for field in (
            "ssl_context_s",
            "connection_init_s",
            "dns_s",
            "tcp_s",
            "proxy_connect_s",
            "tls_s",
        )
    )
    connection = (
        ("reused" if timings["connection_reused"] else "new")
        if "connection_reused" in timings
        else "not_started"
    )
    print(
        dumps(
            {
                "event": result["type"],
                "decision_id": result["decision_id"],
                "tick": result["tick"],
                "worker": record["worker"],
                "transport": timings.get("transport"),
                "connection": connection,
                "queue_s": round(timings["queue_s"], 4),
                "connect_s": round(connect_s, 4),
                "wait_headers_s": round(timings.get("wait_headers_s", 0), 4),
                "api_s": round(timings.get("api_total_s", 0), 4),
                "elapsed_s": round(record["elapsed_s"], 4),
                "action": result.get("action"),
                "error": result.get("error"),
            }
        ),
        flush=True,
    )


def serve(settings):
    """正式帧收发；unity_only 负责暂停采样屏障，核心 worker 只处理决策。"""
    endpoint = (socket.gethostbyname(settings.host), settings.port)
    log = open(settings.log, "a", encoding="utf-8") if settings.log else None
    scheduler = DecisionScheduler(settings, API_KEY)
    adapter = UnitySnapshotAdapter(settings.jev_speed_unit)
    assembler = Reassembler()
    active = {}
    last_control_stamp = 0

    def send_status(sock, task, status, description, result=None, control_stamp=None):
        key = task["simId"], task["id"]
        context = task.get("params", adapter.tasks.get(key, {}))
        params = {"currentRoute": context.get("currentRoute", 0) if isinstance(context, dict) else 0}
        local = task.get("unity_only", {})
        if local.get("decision_id"):
            params["unity_only"] = {k: local[k] for k in ("decision_id", "tick")}
        if result is not None:
            params["unity_only"] = {k: result[k] for k in ("decision_id", "tick")}
            if control_stamp is not None:
                params["unity_only"]["control_timestamp"] = control_stamp
        content = {k: task[k] for k in ("simId", "id", "code")}
        content.update(status=status, description=description, params=params)
        for data in json_packets(content, STATUS):
            sock.sendto(data, endpoint)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as multicast:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((settings.listen_host, settings.listen_port))
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            multicast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            multicast.bind(("", settings.situation_port))
            multicast.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(settings.situation_group) + socket.inet_aton(settings.multicast_interface))
            for record in scheduler.prepare_connections():
                if log:
                    log.write(dumps(record) + "\n"); log.flush()
                print(dumps(record), flush=True)
            print(f"Jev worker listening on {settings.listen_host}:{settings.listen_port}; Unity={endpoint}; situation={settings.situation_group}:{settings.situation_port}", flush=True)
            while True:
                for record in scheduler.collect_results():
                    report_decision(record, log)
                    task = record["request"]["wire_task"]
                    key = task["simId"], task["id"]
                    result = record["result"]
                    if active.get(key) != result["decision_id"]:
                        continue
                    if result["type"] == "decision_error":
                        send_status(sock, task, 3, result["error"], result)
                    else:
                        stamp = timestamp(False)
                        if stamp <= last_control_stamp and last_control_stamp - stamp < 1000:
                            stamp = last_control_stamp + 1
                        last_control_stamp = stamp
                        try:
                            platform = task["params"]["platCode"]
                            data = control_packet(platform, **result["action"], stamp=stamp)
                        except (ValueError, OverflowError) as error:
                            failed = dict(result, type="decision_error", error=str(error))
                            send_status(sock, task, 3, str(error), failed)
                            continue
                        sock.sendto(data, endpoint)
                        send_status(sock, task, 1, "tracking" if record["request"]["type"] == "decision_request" else "stopping", result, stamp)
                readable, _, _ = select.select([sock, multicast], [], [], .01)
                for channel in readable:
                    task = None
                    try:
                        data, source = channel.recvfrom(65507)
                        # Local validation profile: only the configured Unity sender supplies snapshots.
                        if source != endpoint:
                            continue
                        kind, _, _, _ = parse_header(data)
                        if kind == SITUATION:
                            adapter.add_situation(decode_situation(data))
                            continue
                        if channel is multicast or kind not in (TASK, STOP):
                            continue
                        task = assembler.add(data, source)
                        if task is None:
                            continue
                        local = task.get("unity_only", {})
                        event = local.get("event")
                        key = task["simId"], task["id"]
                        if event in ("mission_completed", "stop_completed", "simulation_collision", "simulation_cancelled"):
                            active.pop(key, None)
                            if event in ("mission_completed", "stop_completed"):
                                send_status(sock, task, 2, "completed" if event == "mission_completed" else "stopped")
                            adapter.finish(key)
                            if log:
                                log.write(dumps({"event": event, "notification": task}) + "\n"); log.flush()
                            if settings.exit_on_complete and event == "mission_completed":
                                return
                            continue
                        if not adapter.add_task(task, kind):
                            continue
                        # Single-boat paused profile: admission revokes every older task's output.
                        active.clear()
                        active[key] = local["decision_id"]
                    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
                        print(dumps({"event": "invalid_packet", "error": str(error)}), flush=True)
                        if isinstance(task, dict):
                            local = task.get("unity_only", {})
                            if (all(isinstance(task.get(k), str) and task[k] for k in ("simId", "id", "code"))
                                    and isinstance(local, dict) and isinstance(local.get("decision_id"), str)
                                    and type(local.get("tick")) is int):
                                failed = dict(adapter.identity(task), type="decision_error", error=str(error))
                                send_status(sock, task, 3, str(error), failed)
                try:
                    ready = adapter.ready()
                except (ValueError, KeyError, TypeError, OverflowError) as error:
                    # Reject malformed business data without terminating UDP reception.
                    print(dumps({"event": "invalid_observation", "error": str(error)}), flush=True)
                    adapter.waiting.clear()
                    continue
                for task, packet, error in ready:
                    local = task["unity_only"]
                    if error is None and not valid_request(packet):
                        error = "Invalid decision observation"
                    if error:
                        result = adapter.identity(task)
                        result.update(type="decision_error", error=error)
                        send_status(sock, task, 3, error, result)
                    else:
                        packet["wire_task"] = task
                        if scheduler.submit(packet):
                            send_status(sock, task, 1, "processing")
    finally:
        scheduler.close()
        if log:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=5006)
    parser.add_argument("--situation-group", default="239.255.0.10")
    parser.add_argument("--situation-port", type=int, default=18010)
    parser.add_argument("--multicast-interface", default="127.0.0.1")
    parser.add_argument("--jev-speed-unit", choices=("knots", "mps"), default="knots",
                        help="Jev presentation only; all protocol speeds remain knots")
    parser.add_argument("--model", default="jev-latest")
    parser.add_argument("--endpoint", default="https://api.typesafe.ai/v1/systemone")
    parser.add_argument(
        "--http-transport",
        choices=("persistent", "urllib"),
        default="persistent",
        help="Reuse HTTPS connections (default), or use the original urllib transport",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Concurrent API calls; default 1 for the paused simulation",
    )
    parser.add_argument("--log")
    parser.add_argument("--exit-on-complete", action="store_true")
    settings = parser.parse_args()
    if any(not 1 <= p <= 65535 for p in (settings.port, settings.listen_port, settings.situation_port)):
        parser.error("Ports must be 1..65535")
    if settings.workers < 1:
        parser.error("Workers must be positive")
    try:
        serve(settings)
    except KeyboardInterrupt:
        print("Worker interrupted. Unity remains paused at its next decision boundary.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Jev worker error: {error}\n")


if __name__ == "__main__":
    main()
