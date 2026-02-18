import asyncio
import inspect
import time
from collections.abc import Awaitable, Callable
from typing import Literal
from typing import Any

from .framing import pack, read
from .registry import Registry

Forwarder = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]
AuthProvider = Callable[[dict[str, Any]], bool | dict[str, Any] | Awaitable[bool | dict[str, Any]]]
SecurityMode = Literal["off", "jwt"]


class TcpGatewayServer:
    """tcp gateway server"""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        registry: Registry,
        forwarder: Forwarder | None = None,
        idle_timeout: float = 60.0,
        route_prefix_as_service: bool = False,
        security_mode: SecurityMode | str | None = None,
        require_auth: bool = False,
        auth_provider: AuthProvider | None = None,
    ):
        self.host = host
        self.port = port
        self.registry = registry
        self.forwarder: Forwarder = forwarder or self._default_forwarder
        self.idle_timeout = idle_timeout
        self.route_prefix_as_service = route_prefix_as_service
        self.auth_provider = auth_provider
        # Backward compatibility:
        # - security_mode 우선
        # - 미지정이면 기존 require_auth 플래그 사용
        if security_mode is None:
            self.security_mode: SecurityMode = "jwt" if require_auth else "off"
        else:
            normalized = str(security_mode).strip().lower()
            if normalized not in {"off", "jwt"}:
                raise ValueError(f"unsupported security_mode: {security_mode}")
            self.security_mode = normalized  # type: ignore[assignment]
        self._server: asyncio.AbstractServer | None = None

    @property
    def require_auth(self) -> bool:
        return self.security_mode == "jwt"

    def set_security_mode(
        self,
        mode: SecurityMode | str,
        *,
        auth_provider: AuthProvider | None = None,
    ):
        normalized = str(mode).strip().lower()
        if normalized not in {"off", "jwt"}:
            raise ValueError(f"unsupported security_mode: {mode}")
        self.security_mode = normalized  # type: ignore[assignment]
        if auth_provider is not None:
            self.auth_provider = auth_provider

    @staticmethod
    async def _default_forwarder(msg: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "echo": msg.get("payload")}

    async def _authorize(
        self, msg: dict[str, Any], writer: asyncio.StreamWriter
    ) -> tuple[bool, dict[str, Any]]:
        if self.auth_provider is None:
            return False, {"error": "auth_provider_not_configured"}

        ctx = {
            "auth": dict(msg.get("auth") or {}),
            "message": msg,
            "peer": writer.get_extra_info("peername"),
        }

        result = self.auth_provider(ctx)
        if inspect.isawaitable(result):
            result = await result

        if isinstance(result, dict):
            ok = bool(result.get("ok", False))
            payload = dict(result)
            payload.pop("ok", None)
            return ok, payload

        return bool(result), {}

    @staticmethod
    async def _write_message(writer: asyncio.StreamWriter, msg: dict[str, Any]):
        writer.write(pack(msg))
        await writer.drain()

    @staticmethod
    def _error_response(*, mid: Any, error: str) -> dict[str, Any]:
        return {"id": mid, "type": "res", "ok": False, "error": error}

    @staticmethod
    def _request_context(*, session_id: str, writer: asyncio.StreamWriter) -> dict[str, Any]:
        return {
            "session_id": session_id,
            "peer": writer.get_extra_info("peername"),
        }

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
        authorized = not self.require_auth
        session_id = f"{id(w)}-{time.monotonic_ns()}"

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
                    if self.require_auth and not authorized:
                        await self._write_message(
                            w,
                            self._error_response(mid=mid, error="unauthorized"),
                        )
                        continue
                    await self._write_message(w, {"id": mid, "type": "pong"})
                    continue

                if typ == "auth":
                    ok, payload = await self._authorize(msg, w)
                    if ok:
                        authorized = True
                        await self._write_message(w, {"id": mid, "type": "res", "ok": True, "data": payload})
                    else:
                        await self._write_message(
                            w,
                            self._error_response(
                                mid=mid,
                                error=str(payload.get("error", "auth_failed")),
                            ),
                        )
                    continue

                if self.require_auth and not authorized:
                    await self._write_message(
                        w,
                        self._error_response(mid=mid, error="unauthorized"),
                    )
                    continue

                if typ == "sub":
                    try:
                        await self.registry.sub(msg["topic"], w)
                        await self._write_message(w, {"id": mid, "type": "res", "ok": True})
                    except Exception as e:
                        await self._write_message(
                            w,
                            self._error_response(mid=mid, error=str(e)),
                        )
                    continue

                if typ == "unsub":
                    await self.registry.unsub(msg["topic"], w)
                    await self._write_message(w, {"id": mid, "type": "res", "ok": True})
                    continue

                if typ == "req":
                    try:
                        normalized = self._normalize_req_message(msg)
                        normalized["_rb_ctx"] = self._request_context(session_id=session_id, writer=w)
                        data = await self.forwarder(normalized)
                        resp = {"id": mid, "type": "res", "ok": True, "data": data}
                    except Exception as e:
                        resp = self._error_response(mid=mid, error=str(e))
                    await self._write_message(w, resp)
                    continue

                await self._write_message(
                    w,
                    self._error_response(mid=mid, error="unknown_type"),
                )

        except asyncio.IncompleteReadError:
            pass
        finally:
            idle_task.cancel()
            await self.registry.unsub_all(w)
            w.close()
            await w.wait_closed()

    async def startup(self):
        """start the tcp gateway server"""
        self._server = await asyncio.start_server(self._handle, self.host, self.port)

    async def shutdown(self):
        """shutdown the tcp gateway server"""
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
