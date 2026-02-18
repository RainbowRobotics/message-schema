from __future__ import annotations

from typing import Any

from rb_schemas.sdk import FlowManagerArgs

from .base import RBRby1BaseSDK


class RBRby1StateSDK(RBRby1BaseSDK):
    """Rainbow Robotics RB-Y1 State SDK"""

    def get_state(self, *, flow_manager_args: FlowManagerArgs | None = None) -> dict[str, Any]:
        """
        [RB-Y1 상태 조회]

        Args:
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        robot = self.robot
        if robot is None:
            raise RuntimeError("rby1_not_connected")
        for method in ("get_state", "GetState", "state"):
            fn = getattr(robot, method, None)
            if not callable(fn):
                continue
            result = fn()

            if isinstance(result, dict):
                self._finish_flow(flow_manager_args=flow_manager_args)
                return result
            if hasattr(result, "__dict__"):
                payload = dict(vars(result))
                self._finish_flow(flow_manager_args=flow_manager_args)
                return payload
            payload = {"result": str(result)}
            self._finish_flow(flow_manager_args=flow_manager_args)
            return payload

        return self.whoami(flow_manager_args=flow_manager_args)
