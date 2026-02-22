from __future__ import annotations

import asyncio
from typing import Any

from .router import ModbusRouter, ModbusTable

try:
    from pymodbus.datastore import (
        ModbusDeviceContext,
        ModbusSequentialDataBlock,
        ModbusServerContext,
    )
    from pymodbus.server import StartAsyncTcpServer
except Exception:  # pragma: no cover
    ModbusDeviceContext = None  # type: ignore[assignment]
    ModbusSequentialDataBlock = None  # type: ignore[assignment]
    ModbusServerContext = None  # type: ignore[assignment]
    StartAsyncTcpServer = None  # type: ignore[assignment]


_BaseDataBlock = ModbusSequentialDataBlock if ModbusSequentialDataBlock is not None else object


class _RoutedDataBlock(_BaseDataBlock):  # type: ignore[misc, valid-type]
    def __init__(self, *, table: ModbusTable, size: int, router: ModbusRouter | None):
        if ModbusSequentialDataBlock is None:
            raise RuntimeError("pymodbus is required. Please install `pymodbus`.")
        super().__init__(0, [0] * size)
        self._table = table
        self._router = router

    def getValues(self, address, count=1):  # noqa: N802
        values = super().getValues(address, count)
        if self._router is not None:
            payload_values = values if isinstance(values, (list, tuple)) else None
            self._router.emit(
                table=self._table,
                hook_type="read",
                address=int(address),
                values=list(payload_values or []),
                unit_id=0,
            )
        return values

    def setValues(self, address, values):  # noqa: N802
        super().setValues(address, values)
        if self._router is not None:
            casted = list(values) if not isinstance(values, list) else values
            self._router.emit(
                table=self._table,
                hook_type="write",
                address=int(address),
                values=casted,
                unit_id=0,
            )


class ModbusServer:
    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = 5020,
        router: ModbusRouter | None = None,
        table_size: int = 10_000,
        device_id: int = 0,
        initial_coils: dict[int, bool] | None = None,
        initial_discrete_inputs: dict[int, bool] | None = None,
        initial_holding_registers: dict[int, int] | None = None,
        initial_input_registers: dict[int, int] | None = None,
    ):
        self.host = host
        self.port = port
        self.router = router
        self.table_size = table_size
        self.device_id = device_id
        self.initial_coils = initial_coils or {}
        self.initial_discrete_inputs = initial_discrete_inputs or {}
        self.initial_holding_registers = initial_holding_registers or {}
        self.initial_input_registers = initial_input_registers or {}
        self._task: asyncio.Task[Any] | None = None

    def _apply_initial_values(self, *, block: _RoutedDataBlock, values: dict[int, int | bool]):
        for address, value in values.items():
            idx = int(address)
            if idx < 0 or idx >= self.table_size:
                raise ValueError(
                    f"initial register address out of range: address={idx}, table_size={self.table_size}"
                )
            # pymodbus datastore 기본 주소 체계는 1-based 오프셋으로 동작한다.
            block.setValues(idx + 1, [int(value) if isinstance(value, bool) else int(value)])

    async def startup(self):
        if (
            StartAsyncTcpServer is None
            or ModbusDeviceContext is None
            or ModbusServerContext is None
        ):
            raise RuntimeError("pymodbus is required. Please install `pymodbus`.")

        if self._task is not None and not self._task.done():
            return

        di = _RoutedDataBlock(table="discrete_inputs", size=self.table_size, router=self.router)
        co = _RoutedDataBlock(table="coils", size=self.table_size, router=self.router)
        ir = _RoutedDataBlock(table="input_registers", size=self.table_size, router=self.router)
        hr = _RoutedDataBlock(table="holding_registers", size=self.table_size, router=self.router)

        self._apply_initial_values(block=di, values=self.initial_discrete_inputs)
        self._apply_initial_values(block=co, values=self.initial_coils)
        self._apply_initial_values(block=ir, values=self.initial_input_registers)
        self._apply_initial_values(block=hr, values=self.initial_holding_registers)

        store = ModbusDeviceContext(di=di, co=co, ir=ir, hr=hr)

        context = ModbusServerContext(devices={self.device_id: store}, single=False)
        self._task = asyncio.create_task(
            StartAsyncTcpServer(
                context=context,
                address=(self.host, self.port),
            )
        )
        await asyncio.sleep(0)

    async def shutdown(self):
        if self._task is None:
            return

        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        finally:
            self._task = None
