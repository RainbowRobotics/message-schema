from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable
from dataclasses import dataclass

from flatbuffers.table import Table

from .main import ZenohClient
from .schema import SubscribeOptions


@dataclass(slots=True)
class _Reg:
    topic: str
    cb: Callable
    flatbuffer_obj_t: Table
    opts: SubscribeOptions


class ZenohRouterError(Exception):
    pass


class ZenohRouter:
    def __init__(
        self,
        *,
        prefix: str = "",
        default_options: SubscribeOptions | None = None,
        tags: list[str] | None = None,
        name: str | None = None,
    ):
        self.prefix = prefix.rstrip("/")
        self.client = ZenohClient()
        self.default_options = default_options or SubscribeOptions()
        self.tags = tags or []
        self.name = name or "zenoh"
        self._regs: list[_Reg] = []
        self._handles = []

        self._lock = asyncio.Lock()
        self._started = False
        self._closed = False

    def _join_topic(self, prefix: str, topic: str) -> str:
        if not prefix:
            return topic
        if not topic:  # topic == ""
            return prefix
        return f"{prefix}/{topic.lstrip('/')}"

    # 데코레이터: @router.subscribe("foo/bar")
    def subscribe(
        self,
        topic: str,
        *,
        flatbuffer_obj_t: Table | None = None,
        opts: SubscribeOptions | None = None,
    ):
        full_topic = self._join_topic(self.prefix, topic)

        def deco(func: Callable):
            self._regs.append(
                _Reg(full_topic, func, flatbuffer_obj_t or None, opts or self.default_options)
            )
            return func

        return deco

    def include_router(self, *others: ZenohRouter):
        for other in others:
            for reg in other._regs:
                if any(existing_reg.topic == reg.topic for existing_reg in self._regs):
                    raise ZenohRouterError(f"🚫 Duplicate topic: {reg.topic}")
                self._regs.append(reg)

    async def startup(self):
        async with self._lock:
            # 이미 시작되어 있고, 닫힌 상태가 아니면 무시
            if self._started and not self._closed:
                return
            # 재시작 경로: 닫힘 플래그 해제
            self._closed = False
            self._started = True

            # 구독 선언 (ZenohClient가 내부에서 세션을 lazy-open 한다 가정)
            self._handles = []
            for r in self._regs:
                h = self.client.subscribe(
                    r.topic, r.cb, flatbuffer_obj_t=r.flatbuffer_obj_t, options=r.opts
                )
                self._handles.append(h)

    async def shutdown(self):
        async with self._lock:
            # 이미 닫았으면 재진입 금지 (멱등)
            if self._closed:
                return
            self._closed = True

            # 1) 구독 핸들부터 닫기 (세션보다 먼저)
            for h in self._handles:
                with contextlib.suppress(Exception):
                    h.close()
            self._handles.clear()

            # 2) 세션 닫기 — 이미 닫힌 세션이면 예외 억제
            with contextlib.suppress(Exception):
                self.client.close()
