import asyncio
import time
from collections.abc import Awaitable, Callable
from typing import Any

from .framing import pack, read
from .registry import Registry

Forwarder = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]

class TcpGatewayServer:
    """ tcp gateway server """
    def __init__(
        self,
        *,
        host: str,
        port: int,
        registry: Registry,
        forwarder: Forwarder,
        idle_timeout: float = 60.0,
        route_prefix_as_service: bool = False,
    ):
        self.host = host
        self.port = port
        self.registry = registry
        self.forwarder = forwarder
        self.idle_timeout = idle_timeout
        self.route_prefix_as_service = route_prefix_as_service
        self._server: asyncio.AbstractServer | None = None

    def _normalize_req_message(self, msg: dict[str, Any]) -> dict[str, Any]:
        if not self.route_prefix_as_service:
            return msg

        if isinstance(msg.get("target"), str):
            return msg

        route = msg.get("route")
        if not isinstance(route, str):
            return msg

        svc, sep, tail = route.partition("/")
        if not sep or not svc or not tail:
            return msg

        normalized = dict(msg)
        normalized["target"] = svc
        normalized["route"] = tail
        return normalized

    async def _handle(self, r: asyncio.StreamReader, w: asyncio.StreamWriter):
        last_seen = time.monotonic()

        async def idle_watch():
            while True:
                await asyncio.sleep(5)
                if time.monotonic() - last_seen > self.idle_timeout:
                    w.close()
                    break

        idle_task = asyncio.create_task(idle_watch())

        try:
            while True:
                msg = await read(r)
                last_seen = time.monotonic()

                mid = msg.get("id")
                typ = msg.get("type")

                if typ == "ping":
                    w.write(pack({"id": mid, "type": "pong"}))
                    await w.drain()
                    continue

                if typ == "sub":
                    try:
                        await self.registry.sub(msg["topic"], w)
                        w.write(pack({"id": mid, "type": "res", "ok": True}))
                    except Exception as e:
                        w.write(pack({"id": mid, "type": "res", "ok": False, "error": str(e)}))
                    await w.drain()
                    continue

                if typ == "unsub":
                    await self.registry.unsub(msg["topic"], w)
                    w.write(pack({"id": mid, "type": "res", "ok": True}))
                    await w.drain()
                    continue

                if typ == "req":
                    try:
                        data = await self.forwarder(self._normalize_req_message(msg))
                        resp = {"id": mid, "type": "res", "ok": True, "data": data}
                    except Exception as e:
                        resp = {"id": mid, "type": "res", "ok": False, "error": str(e)}
                    w.write(pack(resp))
                    await w.drain()
                    continue

                w.write(pack({"id": mid, "type": "res", "ok": False, "error": "unknown_type"}))
                await w.drain()

        except asyncio.IncompleteReadError:
            pass
        finally:
            idle_task.cancel()
            await self.registry.unsub_all(w)
            w.close()
            await w.wait_closed()

    async def startup(self):
        """ start the tcp gateway server """
        self._server = await asyncio.start_server(self._handle, self.host, self.port)

    async def shutdown(self):
        """ shutdown the tcp gateway server """
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
