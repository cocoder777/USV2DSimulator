"""一次决策的处理流程，以及有界的后台任务调度。"""
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import threading
import time

from choice_control import build_payload, decode_action
from route_context import RouteContext
from tools.http_transport import JevTransport

PROTOCOL_VERSION = 9
IDENTITY_FIELDS = ("session_id", "episode_id", "run_id", "decision_id", "tick")


def identity(packet):
    return tuple(packet.get(field) for field in IDENTITY_FIELDS)


class DecisionScheduler:
    """默认单工作线程；可增加并发数，但始终只保留一个最新待执行观测。"""

    def __init__(self, settings, key, transport_factory=JevTransport):
        self.settings, self.key = settings, key
        self.max_workers = settings.workers
        self.local = threading.local()
        self.transports = []
        self.transport_lock = threading.Lock()
        self.transport_factory = transport_factory
        self.pool = ThreadPoolExecutor(max_workers=self.max_workers,
                                       thread_name_prefix="jev", initializer=self._init_worker)
        self.inflight = {}
        self.pending = None
        # 只记身份，避免重复数据报造成重复 API 调用；不缓存或重发结果。
        self.seen = OrderedDict()

    def _init_worker(self):
        # HTTPSConnection 不能被多个线程同时使用，每个线程独立复用自己的连接。
        self.local.transport = self.transport_factory()
        self.local.route_context = RouteContext()
        with self.transport_lock:
            self.transports.append(self.local.transport)

    def prepare_connections(self):
        """启动阶段预热所有工作线程；屏障保证每条线程准备自己的连接。"""
        barrier = threading.Barrier(self.max_workers)
        futures = [self.pool.submit(self._prepare_worker, barrier) for _ in range(self.max_workers)]
        return [future.result() for future in futures]

    def _prepare_worker(self, barrier):
        barrier.wait()
        timings = {}
        error = None
        try:
            self.local.transport.prepare_connection(self.settings.endpoint, timings, self.settings.http_transport)
        except Exception as failure:
            # 预连接失败不终止客户端；真实决策仍可按原流程尝试建立连接。
            timings["status"] = "error"
            error = f"{type(failure).__name__}: {failure}"
        return {"event": "preconnect", "worker": threading.current_thread().name,
                "status": timings["status"], "timings": timings, "error": error}

    def submit(self, packet):
        request_id = identity(packet)
        if (request_id in self.seen or request_id in self.inflight
                or (self.pending is not None and request_id == identity(self.pending[0]))):
            return False
        self.seen[request_id] = None
        if len(self.seen) > 256:
            self.seen.popitem(last=False)
        item = (packet, time.monotonic())
        if len(self.inflight) < self.max_workers:
            self._start(item)
        else:
            # 过载时替换尚未执行的旧观测，避免请求无限排队。
            self.pending = item
        return True

    def _start(self, item):
        packet, received_at = item
        self.inflight[identity(packet)] = self.pool.submit(self._execute, packet, received_at)

    def _execute(self, packet, received_at):
        """在工作线程中：构造请求 → 调用 Jev → 解析动作，返回完整记录。"""
        started = time.monotonic()
        timings = {"queue_s": started - received_at}
        envelope = {field: packet[field] for field in IDENTITY_FIELDS}
        envelope["version"] = PROTOCOL_VERSION
        payload = raw = None
        try:
            # 把当前帧的可用信息打包为jev能看懂的信息
            payload = build_payload(packet["observation"], self.settings.model,
                                    self.local.route_context, identity(packet)[:3],
                                    tracking_preference_m=self.settings.tracking_preference_m)
            timings["build_payload_s"] = time.monotonic() - started
            # 正式api请求，默认persistent，复用连接。返回结果后，raw是从响应 JSON 解析成的 Python 字典
            raw = self.local.transport.call(payload, self.key, self.settings.endpoint,
                                            timings, self.settings.http_transport)
            mark = time.monotonic()
            action = decode_action(raw)
            timings["decode_action_s"] = time.monotonic() - mark
            result = dict(envelope, type="decision_result", action=action,
                          action_duration_s=packet["observation"]["planning"]["action_duration_s"])
        except Exception as error:
            result = dict(envelope, type="decision_error", error=f"{type(error).__name__}: {error}")
        return {"event": "decision", "elapsed_s": time.monotonic() - received_at,
                "worker": threading.current_thread().name, "timings": timings,
                "request": packet, "payload": payload, "raw_response": raw, "result": result}

    def collect_results(self):
        '''一个任务完成，腾出空位，启动之前的 pending'''
        records = []
        for request_id, future in list(self.inflight.items()):
            if future.done():
                records.append(future.result())
                del self.inflight[request_id]
        if self.pending is not None and len(self.inflight) < self.max_workers:
            item, self.pending = self.pending, None
            self._start(item)
        # 不要求按请求顺序返回；身份和采样 tick 原样交还 Unity。
        return records

    def close(self):
        self.pending = None
        self.pool.shutdown(wait=True)
        # 所有任务结束后再关闭连接，避免与工作线程抢用连接。
        for transport in self.transports:
            transport.close()
