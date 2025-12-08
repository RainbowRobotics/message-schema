from __future__ import annotations

import contextvars
from collections.abc import Callable
from typing import Any

_current_resolver: contextvars.ContextVar[Callable[[str], Any] | None] = contextvars.ContextVar(
    "rb_global_resolver", default=None
)


def set_current_resolver(resolver: Callable[[str], Any] | None) -> None:
    _current_resolver.set(resolver)


def resolve_from_context(name: str) -> Any:
    res = _current_resolver.get()
    if res is None:
        return None
    return res(name)
