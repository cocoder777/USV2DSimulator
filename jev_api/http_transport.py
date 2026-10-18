"""Measured HTTPS transport. One instance per worker; never retries a POST.

Timings are client observations, not server inference measurements. Proxy CONNECT
and TLS are measured separately. System/environment HTTP proxies are preserved.
"""
import hashlib
import http.client
import json
import socket
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request


class MeasuredHTTPSConnection(http.client.HTTPSConnection):
    def __init__(self, *args, metrics, **kwargs):
        self.metrics = metrics
        started = time.perf_counter()
        super().__init__(*args, **kwargs)
        metrics["connection_init_s"] = time.perf_counter() - started
        self._create_connection = self._connect_socket

    def _connect_socket(self, address, timeout=socket._GLOBAL_DEFAULT_TIMEOUT,
                        source_address=None, **kwargs):
        started = time.perf_counter()
        addresses = socket.getaddrinfo(*address, 0, socket.SOCK_STREAM)
        self.metrics["dns_s"] = time.perf_counter() - started
        started = time.perf_counter()
        last_error = None
        for family, kind, proto, _, target in addresses:
            sock = socket.socket(family, kind, proto)
            try:
                if timeout is not socket._GLOBAL_DEFAULT_TIMEOUT:
                    sock.settimeout(timeout)
                if source_address:
                    sock.bind(source_address)
                sock.connect(target)
                self.metrics["tcp_s"] = time.perf_counter() - started
                return sock
            except OSError as error:
                last_error = error
                sock.close()
        raise last_error or OSError("No addresses resolved")

    def _tunnel(self):
        started = time.perf_counter()
        try:
            return super()._tunnel()
        finally:
            self.metrics["proxy_connect_s"] = time.perf_counter() - started

    def connect(self):
        started = time.perf_counter()
        super().connect()
        self.metrics["tls_s"] = max(0.0, time.perf_counter() - started - sum(
            self.metrics.get(k, 0) for k in ("dns_s", "tcp_s", "proxy_connect_s")))


class _MeasuredHandler(urllib.request.HTTPSHandler):
    def __init__(self, metrics):
        started = time.perf_counter()
        super().__init__()
        metrics["ssl_context_s"] = time.perf_counter() - started
        self.metrics = metrics

    def https_open(self, request):
        def connection(*args, **kwargs):
            return MeasuredHTTPSConnection(*args, metrics=self.metrics, **kwargs)
        return self.do_open(connection, request, context=self._context)


class JevTransport:
    def __init__(self):
        self.connection = None
        self.route = None
        self.context = None

    def close(self):
        if self.connection is not None:
            self.connection.close()
        self.connection = None

    def call(self, payload, key, endpoint, metrics=None, mode="persistent"):
        metrics = metrics if metrics is not None else {}
        started = time.perf_counter()
        mark = started
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False,
                          separators=(",", ":")).encode("utf-8")
        metrics.update(serialize_s=time.perf_counter() - mark,
                       payload_bytes=len(body), payload_sha256=hashlib.sha256(body).hexdigest())
        headers = {"Authorization": "Bearer " + key, "Content-Type": "application/json"}
        try:
            if mode == "urllib":
                metrics.update(transport="urllib", connection_reused=False)
                mark = time.perf_counter()
                opener = urllib.request.build_opener(_MeasuredHandler(metrics))
                metrics["proxy_setup_s"] = max(0.0, time.perf_counter() - mark - metrics["ssl_context_s"])
                request = urllib.request.Request(endpoint, data=body, method="POST", headers=headers)
                mark = time.perf_counter()
                with opener.open(request, timeout=None) as response:
                    metrics["open_s"] = time.perf_counter() - mark
                    metrics["wait_headers_s"] = max(0.0, metrics["open_s"] - sum(
                        metrics.get(k, 0) for k in ("connection_init_s", "dns_s", "tcp_s", "proxy_connect_s", "tls_s")))
                    return self._read(response, metrics)
            if mode != "persistent":
                raise ValueError("Unknown transport: " + mode)
            mark = time.perf_counter()
            url = urllib.parse.urlsplit(endpoint)
            if url.scheme != "https" or not url.hostname or url.username or url.fragment:
                raise ValueError("Expected an HTTPS endpoint without credentials or fragment")
            proxy = None if urllib.request.proxy_bypass(url.hostname) else urllib.request.getproxies().get("https")
            proxy_url = urllib.parse.urlsplit(proxy if "://" in (proxy or "") else "http://" + proxy) if proxy else None
            if proxy_url and (proxy_url.scheme != "http" or proxy_url.username):
                metrics["fallback_reason"] = "Proxy requires urllib transport"
                return self.call(payload, key, endpoint, metrics, mode="urllib")
            route = (endpoint, proxy)
            metrics["proxy_setup_s"] = time.perf_counter() - mark
            metrics.update(transport="persistent", via_proxy=bool(proxy),
                           connection_reused=self.connection is not None and self.route == route and self.connection.sock is not None)
            if not metrics["connection_reused"]:
                self.close()
                mark = time.perf_counter()
                if self.context is None:
                    self.context = ssl.create_default_context()
                metrics["ssl_context_s"] = time.perf_counter() - mark
                self.connection = MeasuredHTTPSConnection(
                    proxy_url.hostname if proxy_url else url.hostname,
                    (proxy_url.port or 80) if proxy_url else (url.port or 443),
                    context=self.context, timeout=None, metrics=metrics)
                if proxy_url:
                    self.connection.set_tunnel(url.hostname, url.port or 443)
                self.route = route
            self.connection.metrics = metrics
            if self.connection.sock is None:
                self.connection.connect()
            path = urllib.parse.urlunsplit(("", "", url.path or "/", url.query, ""))
            mark = time.perf_counter()
            self.connection.request("POST", path, body=body, headers=headers)
            metrics["send_s"] = time.perf_counter() - mark
            mark = time.perf_counter()
            response = self.connection.getresponse()
            metrics["wait_headers_s"] = time.perf_counter() - mark
            with response:
                if not 200 <= response.status < 300:
                    raise RuntimeError(f"Jev returned HTTP {response.status}")
                return self._read(response, metrics)
        except urllib.error.HTTPError as error:
            raise RuntimeError(f"Jev returned HTTP {error.code}") from None
        except Exception:
            # A stale connection can fail after a POST was sent. Let the existing
            # explicit Retry flow decide; replaying silently could duplicate calls.
            self.close()
            raise
        finally:
            metrics["api_total_s"] = time.perf_counter() - started

    @staticmethod
    def _read(response, metrics):
        metrics["http_status"] = response.status
        mark = time.perf_counter()
        data = response.read()
        metrics.update(body_read_s=time.perf_counter() - mark, response_bytes=len(data))
        # Only safe diagnostic headers, never authorization/cookies.
        metrics["response_headers"] = {k.lower(): v for k, v in response.headers.items()
                                       if k.lower() in ("server-timing", "x-request-id", "request-id", "connection", "server", "via")}
        mark = time.perf_counter()
        result = json.loads(data)
        metrics["json_parse_s"] = time.perf_counter() - mark
        return result
