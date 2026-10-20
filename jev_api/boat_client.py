"""Unity 的 UDP 入口；默认暂停决策，HTTPS 在后台线程执行。"""
import argparse
import json
import math
import socket

from decision_worker import DecisionScheduler, IDENTITY_FIELDS, PROTOCOL_VERSION

API_KEY = 'apikey_262fad86dabbdf64bad8847eff8ed10446c_2a28eda5476e1798a4d3c7c843d1c76302706522686efa24d124cf33938ed141'
MAX_PACKET = 65000


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
        transition_time = obs['response']['speed_transition_time_s']
        speed = obs['boat']['speed_mps']
        if (type(transition_time) not in (int, float) or not math.isfinite(transition_time) or transition_time < 0
                or type(speed) not in (int, float) or not math.isfinite(speed)):
            return False
        for field in ('active_target_speed_mps', 'remaining_transition_s'):
            value = obs['response'][field]
            if type(value) not in (int, float) or not math.isfinite(value):
                return False
        if obs['response']['remaining_transition_s'] < 0:
            return False
        for value in (obs['heading_response']['transition_time_s'],
                      obs['heading_response']['active_target_offset_deg'],
                      obs['heading_response']['remaining_transition_s'], obs['boat']['yaw_rate_deg_s'],obs['fixed_dt_s']):
            if type(value) not in (int,float) or not math.isfinite(value):
                return False
        if (obs['heading_response']['transition_time_s'] < 0
                or obs['heading_response']['remaining_transition_s'] < 0 or obs['fixed_dt_s'] <= 0):
            return False
        return (len(obs["action_space"]["speeds_mps"]) == 8
                and len(obs["action_space"]["heading_offsets_deg"]) == 8
                and obs["task"]["planWaypointNum"] == len(obs["task"]["planWaypointList"]))
    except (KeyError, TypeError):
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
    connect_s = sum(timings.get(field, 0) for field in (
        "ssl_context_s", "connection_init_s", "dns_s", "tcp_s", "proxy_connect_s", "tls_s"))
    connection = ("reused" if timings["connection_reused"] else "new") if "connection_reused" in timings else "not_started"
    print(dumps({"event": result["type"], "decision_id": result["decision_id"], "tick": result["tick"],
                 "worker": record["worker"], "transport": timings.get("transport"), "connection": connection,
                 "queue_s": round(timings["queue_s"], 4), "connect_s": round(connect_s, 4),
                 "wait_headers_s": round(timings.get("wait_headers_s", 0), 4),
                 "api_s": round(timings.get("api_total_s", 0), 4), "elapsed_s": round(record["elapsed_s"], 4),
                 "action": result.get("action"), "error": result.get("error")}), flush=True)


def valid_completion(packet):
    return (packet.get("type") == "mission_completed"
            and type(packet.get("version")) is int and packet["version"] == PROTOCOL_VERSION
            and all(isinstance(packet.get(field), str) and packet[field] for field in IDENTITY_FIELDS[:3])
            and type(packet.get("tick")) is int and packet["tick"] >= 0)


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
            print("Preparing HTTPS connections; no decision request is sent...", flush=True)
            for record in scheduler.prepare_connections():
                if log:
                    log.write(dumps(record) + "\n")
                    log.flush()
                print(dumps(record), flush=True)
            print(f"Jev worker listening on {settings.listen_host}:{settings.listen_port}; Unity={endpoint}; workers={settings.workers}", flush=True)
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
    parser.add_argument("--http-transport", choices=("persistent", "urllib"), default="persistent",
                        help="Reuse HTTPS connections (default), or use the original urllib transport")
    parser.add_argument("--workers", type=int, default=1,
                        help="Concurrent API calls; default 1 for the paused simulation")
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
