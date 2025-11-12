from typing import (
    Protocol,
    runtime_checkable,
)

from app.features.move.schema.move import (
    MoveGoalResponse,
    MovePauseResponse,
    MoveResumeResponse,
    MoveStopResponse,
    MoveTargetResponse,
)
from app.features.move.src.domain.move_model import (
    MoveModel,
)


@runtime_checkable
class SlamnavPort(Protocol):
    async def send_move_goal(self, model: MoveModel) -> MoveGoalResponse: ...
    async def send_move_target(self, model: MoveModel) -> MoveTargetResponse: ...
    async def send_move_jog(self, model: MoveModel) -> None: ...
    async def send_move_stop(self, model: MoveModel) -> MoveStopResponse: ...
    async def send_move_pause(self, model: MoveModel) -> MovePauseResponse: ...
    async def send_move_resume(self, model: MoveModel) -> MoveResumeResponse: ...