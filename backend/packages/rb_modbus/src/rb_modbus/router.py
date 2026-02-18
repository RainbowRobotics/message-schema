from __future__ import annotations

import asyncio
import inspect
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

ModbusTable = Literal["coils", "discrete_inputs", "holding_registers", "input_registers"]
HookType = Literal["read", "write"]
HookHandler = Callable[[dict[str, Any]], Any]


@dataclass(slots=True)
class _Hook:
    table: ModbusTable
    hook_type: HookType
    start: int
    count: int
    handler: HookHandler

    def matches(self, *, table: ModbusTable, hook_type: HookType, start: int, count: int) -> bool:
        if self.table != table or self.hook_type != hook_type:
            return False

        lhs_end = self.start + self.count
        rhs_end = start + count
        return not (rhs_end <= self.start or lhs_end <= start)


class ModbusRouter:
    def __init__(self):
        self._hooks: list[_Hook] = []
        self._topic_index: dict[tuple[ModbusTable, HookType], list[_Hook]] = defaultdict(list)

    def on_read(self, table: ModbusTable, *, start: int = 0, count: int = 1):
        return self._register(table=table, hook_type="read", start=start, count=count)

    def on_write(self, table: ModbusTable, *, start: int = 0, count: int = 1):
        return self._register(table=table, hook_type="write", start=start, count=count)

    def include_router(self, other: ModbusRouter):
        for hook in other._hooks:
            self._hooks.append(hook)
            self._topic_index[(hook.table, hook.hook_type)].append(hook)

    def _register(self, *, table: ModbusTable, hook_type: HookType, start: int, count: int):
        def deco(fn: HookHandler):
            hook = _Hook(
                table=table,
                hook_type=hook_type,
                start=start,
                count=max(count, 1),
                handler=fn,
            )
            self._hooks.append(hook)
            self._topic_index[(table, hook_type)].append(hook)
            return fn

        return deco

    def emit(
        self,
        *,
        table: ModbusTable,
        hook_type: HookType,
        address: int,
        values: list[int] | list[bool],
        unit_id: int,
    ):
        payload = {
            "table": table,
            "type": hook_type,
            "address": address,
            "count": len(values),
            "values": values,
            "unit_id": unit_id,
        }

        for hook in self._topic_index.get((table, hook_type), []):
            if not hook.matches(table=table, hook_type=hook_type, start=address, count=len(values)):
                continue
            self._invoke(hook.handler, payload)

    def _invoke(self, handler: HookHandler, payload: dict[str, Any]):
        res = handler(payload)
        if not inspect.isawaitable(res):
            return

        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            asyncio.run(res)
            return

        loop.create_task(res)
