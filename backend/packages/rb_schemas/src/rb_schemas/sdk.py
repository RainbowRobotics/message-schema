from collections.abc import (
    Callable,
)
from dataclasses import (
    dataclass,
)
from typing import Any

# from rb_flow_manager.context import ExecutionContext


@dataclass
class FlowManagerArgs:
    ctx: Any
    args: dict[str, Any]
    done: Callable
