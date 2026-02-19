from __future__ import annotations

import asyncio
import contextlib
import json
import os
import threading
import uuid
from typing import Any

from .framing import pack, read
from .tcp_client_router import TcpClientRouter


class TcpClientError(Exception):
    pass


class TcpClient:
    _instances: dict[
        int, dict[tuple[str, int, float, str | None, bool, str | None], TcpClient]
    ] = {}
    _instances_lock = threading.Lock()

    def __new__(
        cls,
        *,
        host: str,
        port: int,
        timeout: float = 5.0,
        service: str | None = None,
        require_auth: bool = False,
        auth_payload: dict[str, Any] | None = None,
    ):
        pid = os.getpid()
        normalized_service = service.strip("/") if isinstance(service, str) else None
        if auth_payload is None:
            auth_fingerprint = None
        else:
            try:
                auth_fingerprint = json.dumps(auth_payload, sort_keys=True, ensure_ascii=True)
            except TypeError:
                auth_fingerprint = repr(auth_payload)
        key = (
            host,
            port,
            float(timeout),
            normalized_service,
            bool(require_auth),
            auth_fingerprint,
        )

        with cls._instances_lock:
            per_pid = cls._instances.setdefault(pid, {})
            inst = per_pid.get(key)
            if inst is None:
                inst = super().__new__(cls)
                inst._init_done = False
                per_pid[key] = inst
            return inst

    def __init__(
        self,
        *,
        host: str,
        port: int,
        timeout: float = 5.0,
        service: str | None = None,
        require_auth: bool = False,
        auth_payload: dict[str, Any] | None = None,
    ):
        if getattr(self, "_init_done", False):
            return
        self._init_done = True

        self.host = host
        self.port = port
        self.timeout = timeout
        self.service = service.strip("/") if isinstance(service, str) else None
        self.require_auth = require_auth
        self.auth_payload = dict(auth_payload or {}) if auth_payload is not None else None

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._read_task: asyncio.Task | None = None
        self._write_lock = asyncio.Lock()
        self._pending: dict[str, asyncio.Future[dict[str, Any]]] = {}
        self._evt_router = TcpClientRouter()
        self._subscriptions: set[str] = set()
        self._lifecycle_lock = asyncio.Lock()
        self._use_count = 0

    @property
    def connected(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    def include_router(self, router: TcpClientRouter):
        if self.service:
            self._evt_router.include_router(router, prefix=self.service)
            return
        self._evt_router.include_router(router)

    def on(self, topic_pattern: str):
        return self._evt_router.on(self._apply_service_prefix(topic_pattern))

    def _apply_service_prefix(self, route: str) -> str:
        if not self.service:
            return route

        normalized = route.strip("/")
        prefix = f"{self.service}/"
        if normalized.startswith(prefix):
            return normalized
        return f"{prefix}{normalized}"

    async def connect(self):
        async with self._lifecycle_lock:
            self._use_count += 1
            if self.connected:
                return

            try:
                self._reader, self._writer = await asyncio.open_connection(self.host, self.port)
                self._read_task = asyncio.create_task(self._read_loop())
                if self.require_auth or self.auth_payload is not None:
                    res = await self._request({"type": "auth", "auth": self.auth_payload or {}})
                    if not res.get("ok", False):
                        raise TcpClientError(str(res.get("error", "auth_failed")))
            except Exception:
                writer = self._writer
                self._writer = None
                self._reader = None
                read_task = self._read_task
                self._read_task = None
                if read_task is not None:
                    read_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await read_task
                if writer is not None:
                    writer.close()
                    with contextlib.suppress(Exception):
                        await writer.wait_closed()
                self._use_count = max(0, self._use_count - 1)
                raise

    async def disconnect(self):
        async with self._lifecycle_lock:
            if self._use_count > 0:
                self._use_count -= 1
            if self._use_count > 0:
                return

            read_task = self._read_task
            self._read_task = None

            if read_task is not None:
                read_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await read_task

            writer = self._writer
            self._writer = None
            self._reader = None

            if writer is not None:
                writer.close()
                with contextlib.suppress(Exception):
                    await writer.wait_closed()

            self._fail_pending(TcpClientError("tcp_client_disconnected"))

    async def ping(self, timeout: float | None = None) -> bool:
        res = await self._request({"type": "ping"}, timeout=timeout)
        return res.get("type") == "pong"

    async def request(
        self,
        *,
        target: str | None = None,
        route: str,
        payload: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        resolved_target = target
        resolved_route = route

        if resolved_target is None:
            normalized = route.strip("/")
            if self.service:
                service_prefix = f"{self.service}/"
                if not normalized.startswith(service_prefix):
                    raise TcpClientError(
                        "target is required, or route must start with '<service>/' when client.service is set"
                    )
            head, sep, tail = normalized.partition("/")
            if sep and head and tail:
                resolved_target = head
                resolved_route = tail
            else:
                raise TcpClientError(
                    "target is required, or route must be '<service>/<route>' format"
                )

        msg = {
            "type": "req",
            "target": resolved_target,
            "route": resolved_route,
            "payload": payload or {},
        }
        res = await self._request(msg, timeout=timeout)
        if not res.get("ok", False):
            raise TcpClientError(str(res.get("error", "request_failed")))
        return dict(res.get("data") or {})

    async def subscribe(self, topic: str):
        full_topic = self._apply_service_prefix(topic)
        res = await self._request({"type": "sub", "topic": full_topic})
        if not res.get("ok", False):
            raise TcpClientError(str(res.get("error", "subscribe_failed")))
        self._subscriptions.add(full_topic)

    async def unsubscribe(self, topic: str):
        full_topic = self._apply_service_prefix(topic)
        res = await self._request({"type": "unsub", "topic": full_topic})
        if not res.get("ok", False):
            raise TcpClientError(str(res.get("error", "unsubscribe_failed")))
        self._subscriptions.discard(full_topic)

    async def _request(self, msg: dict[str, Any], timeout: float | None = None) -> dict[str, Any]:
        if not self.connected:
            raise TcpClientError("tcp_client_not_connected")

        mid = uuid.uuid4().hex
        msg["id"] = mid

        loop = asyncio.get_running_loop()
        fut: asyncio.Future[dict[str, Any]] = loop.create_future()
        self._pending[mid] = fut

        writer = self._writer
        if writer is None:
            self._pending.pop(mid, None)
            raise TcpClientError("tcp_client_not_connected")

        async with self._write_lock:
            writer.write(pack(msg))
            await writer.drain()

        try:
            return await asyncio.wait_for(fut, timeout=timeout or self.timeout)
        finally:
            self._pending.pop(mid, None)

    async def _read_loop(self):
        reader = self._reader
        if reader is None:
            return

        try:
            while True:
                msg = await read(reader)
                typ = msg.get("type")
                if typ == "res" or typ == "pong":
                    mid = str(msg.get("id") or "")
                    fut = self._pending.get(mid)
                    if fut is not None and not fut.done():
                        fut.set_result(msg)
                    continue

                if typ == "evt":
                    topic = str(msg.get("topic") or "")
                    data = dict(msg.get("data") or {})
                    await self._evt_router.dispatch(topic, data)
                    continue
        except asyncio.CancelledError:
            raise
        except Exception:
            self._fail_pending(TcpClientError("tcp_client_read_loop_terminated"))

    def _fail_pending(self, exc: Exception):
        for fut in list(self._pending.values()):
            if not fut.done():
                fut.set_exception(exc)
        self._pending.clear()
