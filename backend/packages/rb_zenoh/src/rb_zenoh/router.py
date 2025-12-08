from __future__ import (
    annotations,
)

import asyncio
import contextlib
from collections.abc import (
    Callable,
)
from dataclasses import (
    dataclass,
)
from typing import Any

from .client import FBRootReadable, ZenohClient
from .schema import SubscribeOptions


@dataclass(slots=True)
class _Reg:
    topic: str
    cb: Callable
    flatbuffer_obj_t: FBRootReadable | None
    opts: SubscribeOptions
    handle: Any | None = None


@dataclass(slots=True)
class _QueryableReg:
    topic: str
    cb: Callable
    handle: Any | None = None


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
        self._queryables: list[_QueryableReg] = []
        self._lock = asyncio.Lock()
        self._started = False
        self._closed = False

    def _join_topic(self, prefix: str, topic: str) -> str:
        if not prefix:
            return topic
        if not topic:
            return prefix
        return f"{prefix}/{topic.lstrip('/')}"

    # 데코레이터: @router.subscribe("foo/bar")
    def subscribe(
        self,
        topic: str,
        *,
        flatbuffer_obj_t: FBRootReadable | None = None,
        opts: SubscribeOptions | None = None,
    ):
        full_topic = self._join_topic(self.prefix, topic)

        def deco(func: Callable):
            reg = _Reg(full_topic, func, flatbuffer_obj_t or None, opts or self.default_options)
            self._regs.append(reg)

            if self._started and not self._closed:
                handle = self.client.subscribe(
                    reg.topic, reg.cb, flatbuffer_obj_t=reg.flatbuffer_obj_t, options=reg.opts
                )
                reg.handle = handle
            return func

        return deco

    def queryable(self, topic: str):
        full_topic = self._join_topic(self.prefix, topic)

        def deco(func: Callable):
            queryables = _QueryableReg(full_topic, func)
            self._queryables.append(queryables)

            if self._started and not self._closed:
                handle = self.client.queryable(queryables.topic, queryables.cb)
                queryables.handle = handle
            return func

        return deco

    def include_router(self, *others: ZenohRouter, prefix: str | None = None):
        ext_prefix = self._join_topic(self.prefix, (prefix or "").rstrip("/"))

        for other in others:
            for src in other._regs:
                new_topic = self._join_topic(
                    ext_prefix, src.topic if other.prefix == "" else src.topic
                )

                exists_idx = next(
                    (i for i, r in enumerate(self._regs) if r.topic == new_topic), None
                )
                if exists_idx is not None:
                    raise ZenohRouterError(f"🚫 Duplicate topic: {new_topic}")

                reg = _Reg(
                    topic=new_topic,
                    cb=src.cb,
                    flatbuffer_obj_t=src.flatbuffer_obj_t,
                    opts=src.opts,
                )
                self._regs.append(reg)

                if self._started and not self._closed:
                    handle = self.client.subscribe(
                        reg.topic, reg.cb, flatbuffer_obj_t=reg.flatbuffer_obj_t, options=reg.opts
                    )
                    reg.handle = handle

    async def startup(self):
        async with self._lock:
            # 이미 시작되어 있고, 닫힌 상태가 아니면 무시
            if self._started and not self._closed:
                return
            # 재시작 경로: 닫힘 플래그 해제
            self._closed = False
            self._started = True

            for _ in range(10):
                if self.client.session is not None:
                    break
                await asyncio.sleep(0.05)

            self.client.set_loop(asyncio.get_running_loop())

            for r in self._regs:
                if r.handle:  # 이미 동적으로 구독됐을 수 있음
                    continue
                h = self.client.subscribe(
                    r.topic, r.cb, flatbuffer_obj_t=r.flatbuffer_obj_t, options=r.opts
                )
                r.handle = h

    async def shutdown(self):
        async with self._lock:
            if self._closed:
                return
            self._closed = True

            # 구독 핸들부터 닫기
            for r in self._regs:
                if r.handle:
                    with contextlib.suppress(Exception):
                        r.handle.close()
                    r.handle = None

            for q in self._queryables:
                if q.handle:
                    with contextlib.suppress(Exception):
                        q.handle.close()
                    q.handle = None

            # 세션 닫기
            with contextlib.suppress(Exception):
                self.client.close()
