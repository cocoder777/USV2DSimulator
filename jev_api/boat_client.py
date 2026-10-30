"""Unity 的 UDP 入口；默认暂停决策，HTTPS 在后台线程执行。"""

import argparse
import json
import socket
import select
from protocol.frames import (TASK, STATUS, STOP, SITUATION, Reassembler,
    parse_header, decode_situation, json_packets, control_packet, timestamp)

from decision_worker import DecisionScheduler
from protocol.unity_snapshot import UnitySnapshotAdapter
from protocol.status import status_content
from validation import valid_request

API_KEY = "apikey_21784ffc38da031a4f36a81a02fd03f23fe1_5effcf922f6dcbbebd60ba97b3076f8ce1d63d87c7fa405626100d6738286211"


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


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
        observation = adapter.last_observations.get((task['simId'], task['id'])) if status == 1 else None
        content = status_content(task, status, description, adapter.tasks, result, control_stamp, observation)
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
                                description = "position_reached; inspect terminal errors" if event == "mission_completed" and task['code'] == 'TargetImpactMode' else "completed" if event == "mission_completed" else "stopped"
                                send_status(sock, task, 2, description)
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
