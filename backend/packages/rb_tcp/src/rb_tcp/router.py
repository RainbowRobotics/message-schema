from collections.abc import Awaitable, Callable
from typing import Any

Handler = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]

class TcpRouter:
    def __init__(self):
        self._handlers: dict[str, Handler] = {}

    def on(self, route: str):
        def deco(fn: Handler):
            if route in self._handlers:
                raise ValueError(f"duplicate tcp route: {route}")
            self._handlers[route] = fn
            return fn
        return deco

    def include_router(self, other: "TcpRouter"):
        for k, v in other._handlers.items():
            if k in self._handlers:
                raise ValueError(f"duplicate tcp route: {k}")
            self._handlers[k] = v

    async def dispatch(self, route: str, payload: dict[str, Any]) -> dict[str, Any]:
        fn = self._handlers.get(route)
        if fn is None:
            return {"ok": False, "error": "route_not_found", "route": route}
        return await fn(payload)
