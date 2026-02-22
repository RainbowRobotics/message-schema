from __future__ import annotations

import asyncio
import os
import threading
from typing import Any

from pymodbus.client import AsyncModbusTcpClient


class ModbusClientError(Exception):
    pass


class ModbusClient:
    _instances: dict[int, dict[tuple[str, int, float], ModbusClient]] = {}
    _instances_lock = threading.Lock()

    def __new__(cls, *, host: str, port: int = 502, timeout: float = 3.0):
        pid = os.getpid()
        key = (host, port, float(timeout))

        with cls._instances_lock:
            per_pid = cls._instances.setdefault(pid, {})
            inst = per_pid.get(key)
            if inst is None:
                inst = super().__new__(cls)
                inst._init_done = False
                per_pid[key] = inst
            return inst

    def __init__(self, *, host: str, port: int = 502, timeout: float = 3.0):
        if getattr(self, "_init_done", False):
            return
        self._init_done = True

        self.host = host
        self.port = port
        self.timeout = timeout
        self._client: AsyncModbusTcpClient | None = None
        self._lifecycle_lock = asyncio.Lock()
        self._use_count = 0

    @property
    def connected(self) -> bool:
        return bool(self._client and self._client.connected)

    async def connect(self):
        async with self._lifecycle_lock:
            self._use_count += 1

            if self._client is None:
                self._client = AsyncModbusTcpClient(
                    host=self.host, port=self.port, timeout=self.timeout
                )
            if self._client.connected:
                return

            try:
                await self._client.connect()
            except Exception as exc:  # noqa: BLE001
                self._use_count = max(0, self._use_count - 1)
                raise ModbusClientError(
                    f"modbus_connect_error:{type(exc).__name__}:{exc}"
                ) from exc
            if not self._client.connected:
                self._use_count = max(0, self._use_count - 1)
                raise ModbusClientError(f"failed_to_connect_modbus:{self.host}:{self.port}")

    async def disconnect(self):
        async with self._lifecycle_lock:
            if self._use_count > 0:
                self._use_count -= 1
            if self._use_count > 0:
                return

            if self._client is not None:
                self._client.close()
                self._client = None

    async def read_holding_registers(
        self, *, address: int, count: int, unit_id: int = 1
    ) -> list[int]:
        res = await self._call(
            "read_holding_registers", address=address, count=count, unit_id=unit_id
        )
        return list(getattr(res, "registers", []) or [])

    async def read_input_registers(
        self, *, address: int, count: int, unit_id: int = 1
    ) -> list[int]:
        res = await self._call(
            "read_input_registers", address=address, count=count, unit_id=unit_id
        )
        return list(getattr(res, "registers", []) or [])

    async def read_coils(self, *, address: int, count: int, unit_id: int = 1) -> list[bool]:
        res = await self._call("read_coils", address=address, count=count, unit_id=unit_id)
        return list(getattr(res, "bits", []) or [])[:count]

    async def write_register(self, *, address: int, value: int, unit_id: int = 1):
        await self._call("write_register", address=address, value=value, unit_id=unit_id)

    async def write_registers(self, *, address: int, values: list[int], unit_id: int = 1):
        await self._call("write_registers", address=address, values=values, unit_id=unit_id)

    async def write_coil(self, *, address: int, value: bool, unit_id: int = 1):
        await self._call("write_coil", address=address, value=value, unit_id=unit_id)

    async def write_coils(self, *, address: int, values: list[bool], unit_id: int = 1):
        await self._call("write_coils", address=address, values=values, unit_id=unit_id)

    async def _call(self, method: str, **kwargs: Any):
        if self._client is None or not self._client.connected:
            raise ModbusClientError("modbus_client_not_connected")

        unit_id = int(kwargs.pop("unit_id", 1))
        fn = getattr(self._client, method)

        try:
            result = await fn(**kwargs, device_id=unit_id)
        except TypeError:
            pass
        except Exception as exc:  # noqa: BLE001
            raise ModbusClientError(f"modbus_transport_error:{type(exc).__name__}:{exc}") from exc
        else:
            if hasattr(result, "isError") and result.isError():
                self._raise_modbus_error(result)
            return result

        try:
            result = await fn(**kwargs, slave=unit_id)
        except TypeError:
            try:
                result = await fn(**kwargs, unit=unit_id)
            except Exception as exc:  # noqa: BLE001
                raise ModbusClientError(
                    f"modbus_transport_error:{type(exc).__name__}:{exc}"
                ) from exc
        except Exception as exc:  # noqa: BLE001
            raise ModbusClientError(f"modbus_transport_error:{type(exc).__name__}:{exc}") from exc

        if hasattr(result, "isError") and result.isError():
            self._raise_modbus_error(result)
        return result

    def _raise_modbus_error(self, result: Any):
        exc_code = getattr(result, "exception_code", None)
        fn_code = getattr(result, "function_code", None)
        detail = str(result)
        if exc_code is not None or fn_code is not None:
            raise ModbusClientError(
                f"modbus_error:exception_code={exc_code},function_code={fn_code},detail={detail}"
            )
        raise ModbusClientError(f"modbus_error:detail={detail}")
