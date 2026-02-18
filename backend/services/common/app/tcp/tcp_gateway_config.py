from __future__ import annotations

import os
from dataclasses import dataclass


def _as_bool(value: str, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(slots=True)
class TcpGatewayConfig:
    host: str = "0.0.0.0"
    port: int = 9100
    security_mode: str = "off"


@dataclass(slots=True)
class ModbusProxyEnvConfig:
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 1502
    timeout: float = 2.0
    unit_id: int = 1
    auth_enabled: bool = False
    auth_token: str = ""


@dataclass(slots=True)
class CommonTcpEnvConfig:
    gateway: TcpGatewayConfig
    modbus_proxy: ModbusProxyEnvConfig

    @classmethod
    def from_env(cls) -> "CommonTcpEnvConfig":
        return cls(
            gateway=TcpGatewayConfig(
                host=os.getenv("TCP_GATEWAY_HOST", "0.0.0.0"),
                port=int(os.getenv("TCP_GATEWAY_PORT", "9100")),
                security_mode=os.getenv("TCP_GATEWAY_SECURITY_MODE", "off").strip().lower(),
            ),
            modbus_proxy=ModbusProxyEnvConfig(
                enabled=_as_bool(os.getenv("TCP_MODBUS_PROXY_ENABLED", "false")),
                host=os.getenv("TCP_MODBUS_PROXY_HOST", "127.0.0.1"),
                port=int(os.getenv("TCP_MODBUS_PROXY_PORT", "1502")),
                timeout=float(os.getenv("TCP_MODBUS_PROXY_TIMEOUT", "2.0")),
                unit_id=int(os.getenv("TCP_MODBUS_PROXY_UNIT_ID", "1")),
                auth_enabled=_as_bool(os.getenv("TCP_MODBUS_PROXY_AUTH_ENABLED", "false")),
                auth_token=os.getenv("TCP_MODBUS_PROXY_AUTH_TOKEN", "").strip(),
            ),
        )
