# app/socket/router.py
from collections.abc import (
    Callable,
)
from dataclasses import (
    dataclass,
    field,
)
from typing import Any

from pydantic import BaseModel


def _join(*parts: str) -> str:
    parts = tuple(p.strip("/") for p in parts if p)
    return "/".join(parts)


@dataclass
class RBSocketIORoute:
    event: str
    handler: Callable[..., Any]
    path_params: dict[str, list[str]] | None = None
    request_model: BaseModel | None = None
    namespace: str | None = None
    kwargs: dict[str, Any] = field(default_factory=dict)


class RbSocketIORouter:
    def __init__(self, prefix: str = "", namespace: str | None = None):
        self.prefix = prefix.strip("/")
        self.namespace = namespace
        self._routes: list[RBSocketIORoute] = []

    def on(
        self,
        event: str,
        *,
        path_params: dict[str, list[str]] | None = None,
        namespace: str | None = None,
        request_model: BaseModel | None = None,
        **kwargs: Any,
    ):
        def decorator(func: Callable[..., Any]):
            if request_model is not None:
                func = request_model.model_validate_json(func)

            self._routes.append(
                RBSocketIORoute(
                    event=event,
                    handler=func,
                    path_params=path_params,
                    request_model=request_model,
                    namespace=namespace,
                    kwargs=kwargs,
                )
            )
            return func

        return decorator

    @property
    def routes(self) -> list[RBSocketIORoute]:
        return list(self._routes)
