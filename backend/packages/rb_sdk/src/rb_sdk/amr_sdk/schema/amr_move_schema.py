from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Move_Goal import Response_Move_GoalT
from rb_flat_buffers.SLAMNAV.Response_Move_Pause import Response_Move_PauseT
from rb_flat_buffers.SLAMNAV.Response_Move_Resume import Response_Move_ResumeT
from rb_flat_buffers.SLAMNAV.Response_Move_Stop import Response_Move_StopT
from rb_flat_buffers.SLAMNAV.Response_Move_Target import Response_Move_TargetT


@runtime_checkable
class SlamnavMovePort(Protocol):
    @abstractmethod
    async def send_move_goal(self, req_id: str, goal_id: str, method: str, preset: int) -> Response_Move_GoalT: ...
    @abstractmethod
    async def send_move_target(self, req_id: str, goal_pose: list[float], method: str, preset: int) -> Response_Move_TargetT: ...
    @abstractmethod
    async def send_move_jog(self, vx: float, vy: float, wz: float) -> None: ...
    @abstractmethod
    async def send_move_stop(self, req_id: str) -> Response_Move_StopT: ...
    @abstractmethod
    async def send_move_pause(self, req_id: str) -> Response_Move_PauseT: ...
    @abstractmethod
    async def send_move_resume(self, req_id: str) -> Response_Move_ResumeT: ...
