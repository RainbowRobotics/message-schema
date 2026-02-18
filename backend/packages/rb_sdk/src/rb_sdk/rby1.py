from __future__ import annotations

from typing import Any

from rb_schemas.sdk import FlowManagerArgs

from .rby1_sdk.base import RBRby1BaseSDK
from .rby1_sdk.control import RBRby1ControlSDK
from .rby1_sdk.state import RBRby1StateSDK


class RBRby1SDK(RBRby1BaseSDK):
    """Rainbow Robotics RB-Y1 Total SDK"""

    control: RBRby1ControlSDK
    """RB-Y1 제어 관련 SDK 집합"""

    state: RBRby1StateSDK
    """RB-Y1 상태 조회 관련 SDK 집합"""

    def __init__(self, *, endpoint: str, model: str | None = None, auto_connect: bool = True):
        super().__init__(endpoint=endpoint, model=model, auto_connect=auto_connect)

        self.control = RBRby1ControlSDK(endpoint=endpoint, model=model, auto_connect=False)
        self.state = RBRby1StateSDK(endpoint=endpoint, model=model, auto_connect=False)

    @property
    def connected(self) -> bool:
        return super().connected

    def connect(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """
        [RB-Y1 연결]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        return super().connect(flow_manager_args=flow_manager_args)

    def disconnect(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """
        [RB-Y1 연결 해제]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        super().disconnect(flow_manager_args=flow_manager_args)

    def whoami(self, *, flow_manager_args: FlowManagerArgs | None = None) -> dict[str, Any]:
        """
        [RB-Y1 로봇 정보 조회]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        return super().whoami(flow_manager_args=flow_manager_args)

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
        return super().call(
            method,
            *args,
            flow_manager_args=flow_manager_args,
            **kwargs,
        )
