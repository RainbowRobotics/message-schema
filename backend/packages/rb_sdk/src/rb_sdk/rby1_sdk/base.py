from __future__ import annotations

import os
import threading
from typing import Any, ClassVar

from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK
from .client import RBY1ClientSDK
from .config import RBY1Config


class RBRby1BaseSDK(RBBaseSDK):
    """RB-Y1 SDK base with per-process shared adapter."""

    _adapter_by_pid: ClassVar[dict[int, RBY1ClientSDK]] = {}
    _config_by_pid: ClassVar[dict[int, RBY1Config]] = {}
    _rby1_lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self, *, endpoint: str, model: str | None = None, auto_connect: bool = True):
        super().__init__()
        pid = os.getpid()
        cfg = RBY1Config(endpoint=endpoint, model=model)

        with self._rby1_lock:
            existing = self._config_by_pid.get(pid)
            if existing is not None and existing != cfg:
                raise ValueError(
                    "RB-Y1 SDK is shared per process. "
                    "endpoint/model must be same as first initialization."
                )

            if pid not in self._adapter_by_pid:
                self._adapter_by_pid[pid] = RBY1ClientSDK(cfg)
                self._config_by_pid[pid] = cfg

            self._rby1_client = self._adapter_by_pid[pid]

        if auto_connect and not self.connected:
            self.connect()

    @property
    def connected(self) -> bool:
        return self._rby1_client.connected

    @property
    def robot(self) -> Any | None:
        return self._rby1_client.robot

    @staticmethod
    def _finish_flow(*, flow_manager_args: FlowManagerArgs | None):
        if flow_manager_args is not None:
            flow_manager_args.done()

    def connect(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """
        [RB-Y1 연결]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result = self._rby1_client.connect()
        self._finish_flow(flow_manager_args=flow_manager_args)
        return result

    def disconnect(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """
        [RB-Y1 연결 해제]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        self._rby1_client.disconnect()
        self._finish_flow(flow_manager_args=flow_manager_args)

    def whoami(self, *, flow_manager_args: FlowManagerArgs | None = None) -> dict[str, Any]:
        """
        [RB-Y1 로봇 정보 조회]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result = self._rby1_client.whoami()
        self._finish_flow(flow_manager_args=flow_manager_args)
        return result

    def call(
        self,
        method: str,
        *args: Any,
        flow_manager_args: FlowManagerArgs | None = None,
        **kwargs: Any,
    ) -> Any:
        """
        [RB-Y1 원본 SDK 메서드 호출]

        Args:
            method: 호출할 upstream 메서드명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result = self._rby1_client.call(method, *args, **kwargs)
        self._finish_flow(flow_manager_args=flow_manager_args)
        return result
