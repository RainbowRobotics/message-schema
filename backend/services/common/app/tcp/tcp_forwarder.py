from dataclasses import dataclass
import inspect
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING, Any

try:
    from rb_zenoh.client import ZenohClient
except Exception:  # pragma: no cover
    ZenohClient = None  # type: ignore[assignment]

if TYPE_CHECKING:
    from rb_modbus.client import ModbusClient
else:
    ModbusClient = Any

zenoh = ZenohClient() if ZenohClient is not None else None


@dataclass(slots=True)
class ModbusProxyConfig:
    enabled: bool = False
    default_unit_id: int = 1
    auth_enabled: bool = False


ModbusAuthProvider = Callable[[dict[str, Any]], bool | dict[str, Any] | Awaitable[bool | dict[str, Any]]]


def _resolve_target_and_route(msg: dict[str, Any]) -> tuple[str, str]:
    target = msg.get("target")
    route = msg.get("route")

    if isinstance(target, str) and isinstance(route, str):
        return target, route

    if isinstance(route, str):
        svc, sep, tail = route.partition("/")
        if sep and svc and tail:
            return svc, tail

    raise ValueError("target/route required. or route must be '<service>/<route>'")


async def _forward_to_modbus(
    *,
    route: str,
    payload: dict[str, Any],
    client: ModbusClient,
    default_unit_id: int,
) -> dict[str, Any]:
    unit_id = int(payload.get("unit_id", default_unit_id))

    if route == "read_holding_registers":
        values = await client.read_holding_registers(
            address=int(payload["address"]),
            count=int(payload["count"]),
            unit_id=unit_id,
        )
        return {"values": values}

    if route == "read_input_registers":
        values = await client.read_input_registers(
            address=int(payload["address"]),
            count=int(payload["count"]),
            unit_id=unit_id,
        )
        return {"values": values}

    if route == "read_coils":
        values = await client.read_coils(
            address=int(payload["address"]),
            count=int(payload["count"]),
            unit_id=unit_id,
        )
        return {"values": values}

    if route == "write_register":
        await client.write_register(
            address=int(payload["address"]),
            value=int(payload["value"]),
            unit_id=unit_id,
        )
        return {"ok": True}

    if route == "write_registers":
        await client.write_registers(
            address=int(payload["address"]),
            values=[int(v) for v in list(payload["values"])],
            unit_id=unit_id,
        )
        return {"ok": True}

    if route == "write_coil":
        await client.write_coil(
            address=int(payload["address"]),
            value=bool(payload["value"]),
            unit_id=unit_id,
        )
        return {"ok": True}

    if route == "write_coils":
        await client.write_coils(
            address=int(payload["address"]),
            values=[bool(v) for v in list(payload["values"])],
            unit_id=unit_id,
        )
        return {"ok": True}

    raise ValueError(f"unsupported_modbus_route:{route}")


def build_forward_to_service(
    *,
    modbus_client: ModbusClient | None = None,
    modbus_proxy_config: ModbusProxyConfig | None = None,
    modbus_auth_provider: ModbusAuthProvider | None = None,
):
    cfg = modbus_proxy_config or ModbusProxyConfig()
    modbus_authed_sessions: set[str] = set()

    async def _check_modbus_auth(session_id: str, payload: dict[str, Any]):
        if not cfg.auth_enabled:
            return

        token = payload.get("token")
        if modbus_auth_provider is None:
            raise ValueError("modbus_auth_provider_not_configured")

        verdict = modbus_auth_provider({"token": token, "payload": payload, "session_id": session_id})
        if inspect.isawaitable(verdict):
            verdict = await verdict

        ok = bool(verdict.get("ok", False)) if isinstance(verdict, dict) else bool(verdict)
        if not ok:
            raise ValueError("modbus_auth_failed")

        modbus_authed_sessions.add(session_id)

    async def _forward(msg: dict[str, Any]) -> dict[str, Any]:
        target, route = _resolve_target_and_route(msg)
        payload = msg.get("payload") or {}
        meta = dict(msg.get("_rb_ctx") or {})
        session_id = str(meta.get("session_id") or "")

        if not isinstance(payload, dict):
            raise ValueError("payload must be object")

        if target == "modbus":
            if not cfg.enabled:
                raise ValueError("modbus_proxy_disabled")
            if modbus_client is None:
                raise ValueError("modbus_proxy_client_not_configured")
            if not session_id:
                raise ValueError("modbus_proxy_session_missing")

            if route == "auth":
                await _check_modbus_auth(session_id, payload)
                return {"ok": True}

            if cfg.auth_enabled and session_id not in modbus_authed_sessions:
                raise ValueError("modbus_unauthorized")

            return await _forward_to_modbus(
                route=route,
                payload=payload,
                client=modbus_client,
                default_unit_id=cfg.default_unit_id,
            )

        # ✅ Zenoh query key 규칙
        # rpc/<service>/<route>
        key = f"rpc/{target}/{route}"
        if zenoh is None:
            raise RuntimeError("zenoh_client_not_available")
        res = await zenoh.query(keyexpr=key, payload=payload, timeout=3.0)
        return res

    return _forward


async def forward_to_service(msg: dict[str, Any]) -> dict[str, Any]:
    # backward compatibility for older imports/tests
    return await build_forward_to_service()(msg)


async def forward_to_service_via_zenoh(msg: dict[str, Any]) -> dict[str, Any]:
    # explicit zenoh-only alias
    target, route = _resolve_target_and_route(msg)
    payload = msg.get("payload") or {}

    if not isinstance(payload, dict):
        raise ValueError("payload must be object")

    # ✅ Zenoh query key 규칙
    # rpc/<service>/<route>
    key = f"rpc/{target}/{route}"
    if zenoh is None:
        raise RuntimeError("zenoh_client_not_available")

    # 아래는 너희 ZenohClient의 query API에 맞춰 호출만 바꾸면 됨.
    # 핵심: query 보내고 reply를 dict로 받아 반환
    res = await zenoh.query(keyexpr=key, payload=payload, timeout=3.0)
    return res
