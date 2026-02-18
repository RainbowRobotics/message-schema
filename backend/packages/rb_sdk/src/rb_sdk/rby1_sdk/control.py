from __future__ import annotations

from typing import Any

from rb_schemas.sdk import FlowManagerArgs

from .base import RBRby1BaseSDK


class RBRby1ControlSDK(RBRby1BaseSDK):
    """Rainbow Robotics RB-Y1 Control SDK"""

    def _call_existing(self, *method_names: str) -> Any:
        robot = self.robot
        if robot is None:
            raise RuntimeError("rby1_not_connected")
        for method in method_names:
            fn = getattr(robot, method, None)
            if callable(fn):
                return fn()
        raise RuntimeError(f"rby1 methods not found: {method_names}")

    def invoke(
        self,
        method: str,
        *args: Any,
        flow_manager_args: FlowManagerArgs | None = None,
        **kwargs: Any,
    ) -> Any:
        """
        [RB-Y1 원본 제어 메서드 호출]

        Args:
            method: 호출할 upstream 제어 메서드명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        return self.call(
            method,
            *args,
            flow_manager_args=flow_manager_args,
            **kwargs,
        )

    def power_on(self, *, flow_manager_args: FlowManagerArgs | None = None) -> Any:
        """
        [RB-Y1 전원 활성화]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result = self._call_existing("power_on", "PowerOn", "enable_power")
        self._finish_flow(flow_manager_args=flow_manager_args)
        return result

    def servo_on(self, *, flow_manager_args: FlowManagerArgs | None = None) -> Any:
        """
        [RB-Y1 서보 활성화]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result = self._call_existing("servo_on", "ServoOn", "enable_servo")
        self._finish_flow(flow_manager_args=flow_manager_args)
        return result
