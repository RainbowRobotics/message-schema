from __future__ import annotations

import contextlib
from collections.abc import Callable
from dataclasses import dataclass

from .main import ZenohClient
from .schema import SubscribeOptions


@dataclass(slots=True)
class _Reg:
    topic: str
    cb: Callable
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

    def _join_topic(self, prefix: str, topic: str) -> str:
        if not prefix:
            return topic
        if not topic:  # topic == ""
            return prefix
        return f"{prefix}/{topic.lstrip('/')}"

    # 데코레이터: @router.subscribe("foo/bar")
    def subscribe(self, topic: str, opts: SubscribeOptions | None = None):
        full_topic = self._join_topic(self.prefix, topic)

        def deco(func: Callable):
            self._regs.append(_Reg(full_topic, func, opts or self.default_options))
            return func

        return deco

    def include_router(self, *others: ZenohRouter):
        for other in others:
            for reg in other._regs:
                if any(existing_reg.topic == reg.topic for existing_reg in self._regs):
                    raise ZenohRouterError(f"🚫 Duplicate topic: {reg.topic}")
                self._regs.append(reg)

    async def startup(self):
        for r in self._regs:
            h = self.client.subscribe(r.topic, r.cb, options=r.opts)
            self._handles.append(h)

    async def shutdown(self):
        for h in self._handles:
            with contextlib.suppress(Exception):
                h.close()
        self._handles.clear()
        self.client.close()
