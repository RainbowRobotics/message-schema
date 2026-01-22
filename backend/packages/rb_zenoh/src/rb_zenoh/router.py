from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, TypeVar, overload

from .client import FBRootReadable, ZenohClient
from .schema import SubscribeOptions

TReq = TypeVar("TReq")
TRes = TypeVar("TRes")

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
    cb: Callable[..., Any]
    flatbuffer_req_t: Any | None = None
    flatbuffer_res_buf_size: int | None = None
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

    # ✅ FlatBuffer req를 "객체 타입"으로 받는 모드
    @overload
    def queryable(
        self,
        topic: str,
        *,
        flatbuffer_req_t: type[TReq],
        flatbuffer_res_buf_size: int,
    ) -> Callable[[Callable[[TReq], TRes] | Callable[[TReq, dict[str, str]], TRes]], Callable[..., TRes]]:
        ...

    # ✅ req 없는 모드 (params만 혹은 아무것도 안 받는 콜백)
    @overload
    def queryable(
        self,
        topic: str,
        *,
        flatbuffer_req_t: None = None,
        flatbuffer_res_buf_size: int | None = None,
    ) -> Callable[[Callable[[], TRes] | Callable[[dict[str, str]], TRes]], Callable[..., TRes]]:
        ...


    # 데코레이터: @router.queryable("file/get")
    def queryable(
        self,
        topic: str,
        *,
        flatbuffer_req_t=None,
        flatbuffer_res_buf_size=None,
    ):
        full_topic = self._join_topic(self.prefix, topic)

        def deco(func):
            reg = _QueryableReg(
                topic=full_topic,
                cb=func,
                flatbuffer_req_t=flatbuffer_req_t,
                flatbuffer_res_buf_size=flatbuffer_res_buf_size,
            )
            self._queryables.append(reg)

            if self._started and not self._closed:
                reg.handle = self.client.queryable(
                    reg.topic,
                    reg.cb,
                    flatbuffer_req_T_class=reg.flatbuffer_req_t,
                    flatbuffer_res_buf_size=reg.flatbuffer_res_buf_size,
                )
            return func

        return deco

    def include_router(self, *others: ZenohRouter, prefix: str | None = None):
        ext_prefix = self._join_topic(self.prefix, (prefix or "").rstrip("/"))

        for other in others:
            # subscribe 복제
            for src in other._regs:
                new_topic = self._join_topic(ext_prefix, src.topic)

                exists = any(r.topic == new_topic for r in self._regs)
                if exists:
                    raise ZenohRouterError(f"🚫 Duplicate topic: {new_topic}")

                reg = _Reg(
                    topic=new_topic,
                    cb=src.cb,
                    flatbuffer_obj_t=src.flatbuffer_obj_t,
                    opts=src.opts,
                )
                self._regs.append(reg)

                if self._started and not self._closed:
                    reg.handle = self.client.subscribe(
                        reg.topic, reg.cb, flatbuffer_obj_t=reg.flatbuffer_obj_t, options=reg.opts
                    )

            # queryable 복제
            for src in other._queryables:
                new_topic = self._join_topic(ext_prefix, src.topic)

                exists = any(q.topic == new_topic for q in self._queryables)
                if exists:
                    raise ZenohRouterError(f"🚫 Duplicate queryable: {new_topic}")

                qreg = _QueryableReg(
                    topic=new_topic,
                    cb=src.cb,
                    flatbuffer_req_t=src.flatbuffer_req_t,
                    flatbuffer_res_buf_size=src.flatbuffer_res_buf_size,
                )
                self._queryables.append(qreg)

                if self._started and not self._closed:
                    qreg.handle = self.client.queryable(
                                    qreg.topic,
                                    qreg.cb,
                                    flatbuffer_req_T_class=qreg.flatbuffer_req_t,
                                    flatbuffer_res_buf_size=qreg.flatbuffer_res_buf_size,
                                )


    async def startup(self):
        async with self._lock:
            if self._started and not self._closed:
                return

            self._closed = False
            self._started = True

            for _ in range(10):
                if self.client.session is not None:
                    break
                await asyncio.sleep(0.05)

            self.client.set_loop(asyncio.get_running_loop())

            # subscribe 등록
            for r in self._regs:
                if r.handle:
                    continue
                r.handle = self.client.subscribe(
                    r.topic, r.cb, flatbuffer_obj_t=r.flatbuffer_obj_t, options=r.opts
                )

            # queryable 등록
            for q in self._queryables:
                if q.handle:
                    continue
                q.handle = self.client.queryable(q.topic, q.cb, flatbuffer_req_T_class=q.flatbuffer_req_t, flatbuffer_res_buf_size=q.flatbuffer_res_buf_size)

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

            # ✅ queryable은 undeclare로 내리기
            for q in self._queryables:
                if q.handle:
                    # zenoh-python 핸들은 보통 .undeclare()
                    with contextlib.suppress(Exception):
                        if hasattr(q.handle, "undeclare"):
                            q.handle.undeclare()
                        elif hasattr(q.handle, "close"):
                            q.handle.close()
                    q.handle = None

                # 안전망: client 쪽 맵에서도 제거(등록되어 있으면)
                with contextlib.suppress(Exception):
                    self.client.undeclare_queryable(q.topic)

            # 세션 닫기
            with contextlib.suppress(Exception):
                self.client.close()
