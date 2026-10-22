"""Unity 的 UDP 入口；默认暂停决策，HTTPS 在后台线程执行。"""

import argparse
import json
import math
import socket

from decision_worker import DecisionScheduler, IDENTITY_FIELDS
from tools.waypoint_geometry import WaypointGeometry

API_KEY = "apikey_262fad86dabbdf64bad8847eff8ed10446c_2a28eda5476e1798a4d3c7c843d1c76302706522686efa24d124cf33938ed141"
MAX_PACKET = 65000


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
            or not 1 <= task["next_waypoint_index"] < task["planWaypointNum"]
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


def receive_packet(sock, endpoint):
    """只接收约定 Unity 地址的合法 JSON；无消息时返回 None。"""
    try:
        data, source = sock.recvfrom(MAX_PACKET + 1)
    except socket.timeout:
        return None
    except OSError as error:
        if getattr(error, "winerror", None) in (10040, 10054):
            return None
        raise
    if source != endpoint or len(data) > MAX_PACKET:
        return None
    try:
        packet = json.loads(data.decode("utf-8"), parse_constant=reject_constant)
        return packet if isinstance(packet, dict) else None
    except (ValueError, UnicodeError):
        return None


def reject_constant(value):
    raise ValueError("Non-finite JSON number: " + value)


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


def valid_completion(packet):
    return (
        packet.get("type") == "mission_completed"
        and all(
            isinstance(packet.get(field), str) and packet[field]
            for field in IDENTITY_FIELDS[:3]
        )
        and type(packet.get("tick")) is int
        and packet["tick"] >= 0
    )


def record_completion(packet, log):
    record = {"event": "completed", "notification": packet}
    if log:
        log.write(dumps(record) + "\n")
        log.flush()
    print(dumps(record), flush=True)


def serve(settings):
    endpoint = (socket.gethostbyname(settings.host), settings.port)
    log = open(settings.log, "a", encoding="utf-8") if settings.log else None
    scheduler = DecisionScheduler(settings, API_KEY)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((settings.listen_host, settings.listen_port))
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            sock.settimeout(0.01)
            print(
                "Preparing HTTPS connections; no decision request is sent...",
                flush=True,
            )
            for record in scheduler.prepare_connections():
                if log:
                    log.write(dumps(record) + "\n")
                    log.flush()
                print(dumps(record), flush=True)
            print(
                f"Jev worker listening on {settings.listen_host}:{settings.listen_port}; Unity={endpoint}; workers={settings.workers}",
                flush=True,
            )
            while True:
                # API 不阻塞 UDP；每份结果只发一次，不主动重发，也不等待 ACK。
                # 检查有没有完成的决策，如果完成的通过UDP交给unity
                for record in scheduler.collect_results():
                    report_decision(record, log)
                    sock.sendto(dumps(record["result"]).encode("utf-8"), endpoint)
                # 接收unity消息
                packet = receive_packet(sock, endpoint)
                if packet is None:
                    continue
                # 如果收到的是决策请求
                if valid_request(packet):
                    scheduler.submit(packet)
                # 完成通知始终记录，参数只决定是否退出客户端。
                elif valid_completion(packet):
                    record_completion(packet, log)
                    if settings.exit_on_complete:
                        return
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
    if any(not 1 <= p <= 65535 for p in (settings.port, settings.listen_port)):
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
