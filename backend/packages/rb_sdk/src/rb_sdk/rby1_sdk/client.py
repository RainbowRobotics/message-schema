from __future__ import annotations

import contextlib
import importlib
from typing import Any

from .config import RBY1Config
from .exceptions import (
    RBY1SDKConnectionError,
    RBY1SDKError,
    RBY1SDKImportError,
)


def _load_rby1_module():
    for mod_name in ("rby1_sdk", "rby1"):
        try:
            return importlib.import_module(mod_name)
        except ModuleNotFoundError:
            continue
    raise RBY1SDKImportError(
        "rby1 sdk is not installed. install with: `pip install rby1-sdk`"
    )


class RBY1ClientSDK:
    """Thin adapter over official rby1 sdk."""

    def __init__(self, config: RBY1Config):
        self.config = config
        self._module: Any | None = None
        self._robot: Any | None = None

    @property
    def robot(self) -> Any | None:
        return self._robot

    @property
    def connected(self) -> bool:
        if self._robot is None:
            return False
        checker = getattr(self._robot, "is_connected", None)
        if callable(checker):
            try:
                return bool(checker())
            except Exception:
                return False
        return True

    def connect(self) -> Any:
        self._module = _load_rby1_module()

        create_robot = getattr(self._module, "create_robot", None)
        if not callable(create_robot):
            raise RBY1SDKError("unsupported rby1 sdk: `create_robot` is missing")

        if self.config.model is not None:
            robot = create_robot(self.config.endpoint, self.config.model)
        else:
            robot = create_robot(self.config.endpoint)

        connect_fn = getattr(robot, "connect", None)
        if callable(connect_fn):
            ok = connect_fn()
            if ok is False:
                raise RBY1SDKConnectionError(
                    f"failed_to_connect_rby1:{self.config.endpoint}"
                )

        self._robot = robot
        return robot

    def disconnect(self):
        if self._robot is None:
            return
        for method_name in ("disconnect", "close"):
            fn = getattr(self._robot, method_name, None)
            if callable(fn):
                with contextlib.suppress(Exception):
                    fn()
                break
        self._robot = None

    def whoami(self) -> dict[str, Any]:
        if self._robot is None:
            raise RBY1SDKConnectionError("rby1_not_connected")

        for method_name in ("get_robot_info", "whoami", "robot_info"):
            fn = getattr(self._robot, method_name, None)
            if callable(fn):
                result = fn()
                if isinstance(result, dict):
                    return result
                if hasattr(result, "__dict__"):
                    return dict(vars(result))
                return {"result": str(result)}

        return {
            "endpoint": self.config.endpoint,
            "model": self.config.model,
            "connected": self.connected,
        }

    def call(self, method: str, *args: Any, **kwargs: Any) -> Any:
        if self._robot is None:
            raise RBY1SDKConnectionError("rby1_not_connected")

        fn = getattr(self._robot, method, None)
        if not callable(fn):
            raise RBY1SDKError(f"rby1_method_not_found:{method}")
        return fn(*args, **kwargs)
