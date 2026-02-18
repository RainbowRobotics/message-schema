from __future__ import annotations

import asyncio
import fnmatch
import inspect
from collections import defaultdict
from collections.abc import Awaitable, Callable
from typing import Any

EventHandler = Callable[[str, dict[str, Any]], Awaitable[None] | None]


def _join_prefix(prefix: str, route: str) -> str:
    left = prefix.strip("/")
    right = route.strip("/")
    if not left:
        return right
    if not right:
        return left
    return f"{left}/{right}"


class TcpClientRouter:
    def __init__(self):
        self._handlers: dict[str, list[EventHandler]] = defaultdict(list)

    def on(self, topic_pattern: str):
        def deco(fn: EventHandler):
            self._handlers[topic_pattern].append(fn)
            return fn

        return deco

    def include_router(self, other: TcpClientRouter, *, prefix: str | None = None):
        for pattern, handlers in other._handlers.items():
            joined = _join_prefix(prefix or "", pattern) if prefix else pattern
            self._handlers[joined].extend(handlers)

    async def dispatch(self, topic: str, payload: dict[str, Any]):
        for pattern, handlers in self._handlers.items():
            if not fnmatch.fnmatchcase(topic, pattern):
                continue

            for handler in handlers:
                res = handler(topic, payload)
                if inspect.isawaitable(res):
                    await res

    def topics(self) -> set[str]:
        return set(self._handlers.keys())

    def dispatch_nowait(self, topic: str, payload: dict[str, Any]):
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        loop.create_task(self.dispatch(topic, payload))
