from __future__ import annotations

from rb_tcp.gateway_server import TcpGatewayServer
from rb_tcp.registry import Registry

from .tcp_forwarder import ModbusProxyConfig, build_forward_to_service
from .tcp_gateway_config import CommonTcpEnvConfig


def build_tcp_gateway_stack(*, cfg: CommonTcpEnvConfig) -> tuple[TcpGatewayServer, Registry, object | None]:
    modbus_proxy_client = None
    if cfg.modbus_proxy.enabled:
        from rb_modbus.client import ModbusClient

        modbus_proxy_client = ModbusClient(
            host=cfg.modbus_proxy.host,
            port=cfg.modbus_proxy.port,
            timeout=cfg.modbus_proxy.timeout,
        )

    forwarder = build_forward_to_service(
        modbus_client=modbus_proxy_client,
        modbus_proxy_config=ModbusProxyConfig(
            enabled=cfg.modbus_proxy.enabled,
            default_unit_id=cfg.modbus_proxy.unit_id,
            auth_enabled=cfg.modbus_proxy.auth_enabled,
        ),
        modbus_auth_provider=(
            (
                lambda ctx: bool(cfg.modbus_proxy.auth_token)
                and str(ctx.get("token") or "") == cfg.modbus_proxy.auth_token
            )
            if cfg.modbus_proxy.auth_enabled
            else None
        ),
    )

    registry = Registry()
    gateway = TcpGatewayServer(
        host=cfg.gateway.host,
        port=cfg.gateway.port,
        registry=registry,
        forwarder=forwarder,
        route_prefix_as_service=True,
        security_mode=cfg.gateway.security_mode,
    )
    return gateway, registry, modbus_proxy_client
