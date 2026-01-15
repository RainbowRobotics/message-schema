"""
[AMR 이동 모델]
"""
import uuid
from dataclasses import (
    dataclass,
    field,
)
from datetime import (
    UTC,
    datetime,
)
from enum import Enum
from typing import Any

from rb_utils.service_exception import (
    ServiceException,
)

from app.features.move.schema.move_api import (
    Request_Move_CircularPD,
    Request_Move_GoalPD,
    Request_Move_JogPD,
    Request_Move_RotatePD,
    Request_Move_TargetPD,
    Request_Move_XLinearPD,
)
from app.schema.amr import AmrResponseStatusEnum


# === Enums ==========================================================
class AmrMoveMethodEnum(str, Enum):
    """
    [AMR 이동 방법]
    """
    PP = "pp"
    HPP = "hpp"

class AmrMoveCommandEnum(str, Enum):
    """
    [AMR 이동 명령]
    """
    MOVE_GOAL     = "goal"
    MOVE_TARGET   = "target"
    MOVE_JOG      = "jog"
    MOVE_STOP     = "stop"
    MOVE_PAUSE    = "pause"
    MOVE_RESUME   = "resume"
    MOVE_X_LINEAR  = "xlinear"
    MOVE_CIRCULAR = "circular"
    MOVE_ROTATE   = "rotate"


# === Model ==========================================================
@dataclass
class MoveModel:
    """
    [AMR 이동 모델]
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    command: AmrMoveCommandEnum | None = None
    status: AmrResponseStatusEnum = field(default=AmrResponseStatusEnum.PENDING)
    created_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    update_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    method: AmrMoveMethodEnum | None = None
    preset: int | None = None
    result: str = None
    message: str | None = None

    # moveGoal
    goal_id: str | None = None

    # moveTarget
    goal_pose: list[float] | None = None

    # moveJog
    vx: float | None = None
    vy: float | None = None
    wz: float | None = None

    # profileMove / others
    target: float | None = None
    speed: float | None = None
    direction: str | None = None

    # response
    cur_pose: list[float] | None = None
    cur_vel: list[float] | None = None
    remaining_time: float | None = None
    goal_name: str | None = None
    bat_percent: int | None = None
    map_name: str | None = None

    def set_move_goal(self, req: Request_Move_GoalPD):
        """
        [AMR 이동 모델 설정]
        """
        self.command = AmrMoveCommandEnum.MOVE_GOAL
        self.method = req.method if req.method is not None else AmrMoveMethodEnum.PP.value
        self.preset = req.preset if req.preset is not None else 0
        self.goal_id = req.goalId
        self.update_at = datetime.now(UTC)

    def set_move_target(self, req: Request_Move_TargetPD):
        """
        - req: Request_Move_TargetPD
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_TARGET
        self.method = req.method if req.method is not None else AmrMoveMethodEnum.PP.value
        self.preset = req.preset if req.preset is not None else 0
        self.goal_pose = req.goalPose
        self.update_at = datetime.now(UTC)

    def set_move_jog(self, req: Request_Move_JogPD):
        """
        - req: Request_Move_JogPD
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_JOG
        self.vx = req.vx if req.vx is not None else 0
        self.vy = req.vy if req.vy is not None else 0
        self.wz = req.wz if req.wz is not None else 0
        self.update_at = datetime.now(UTC)

    def set_move_x_linear(self, req: Request_Move_XLinearPD):
        """
        - req: Request_Move_XLinearPD
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_X_LINEAR
        self.target = req.target
        self.speed = req.speed
        self.update_at = datetime.now(UTC)

    def set_move_circular(self, req: Request_Move_CircularPD):
        """
        - req: Request_Move_CircularPD
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_CIRCULAR
        self.direction = req.direction
        self.target = req.target
        self.speed = req.speed
        self.update_at = datetime.now(UTC)

    def set_move_rotate(self, req: Request_Move_RotatePD):
        """
        - req: Request_Move_RotatePD
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_ROTATE
        self.target = req.target
        self.speed = req.speed
        self.update_at = datetime.now(UTC)

    def set_move_stop(self):
        """
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_STOP
        self.update_at = datetime.now(UTC)

    def set_move_pause(self):
        """
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_PAUSE
        self.update_at = datetime.now(UTC)

    def set_move_resume(self):
        """
        - return: None
        """
        self.command = AmrMoveCommandEnum.MOVE_RESUME
        self.update_at = datetime.now(UTC)

    def status_change(self, status: str) -> None:
        """
        - status: str
        - return: None
        """
        if not self.id:
            raise ServiceException("ID가 없습니다", status_code=400)
        self.status = self.parse_status(status)
        self.update_at = datetime.now(UTC)

    def result_change(self, result: str) -> None:
        """
        - result: str
        - return: None
        """
        if not self.id:
            raise ServiceException("ID가 없습니다", status_code=400)
        self.result = result
        self.update_at = datetime.now(UTC)

    @staticmethod
    def parse_status(value: str) -> AmrResponseStatusEnum:
        """
        - value: str
        - return: AmrResponseStatusEnum
        """
        try:
            return AmrResponseStatusEnum(value)
        except ValueError:
            return AmrResponseStatusEnum.UNKNOWN

    def check_variables(self) -> None:
        """
        - return: None
        """
        if self.command == AmrMoveCommandEnum.MOVE_GOAL:
            if self.goal_id == "":
                raise ServiceException("goal_id 값이 없습니다", status_code=400)

        elif self.command == AmrMoveCommandEnum.MOVE_TARGET:
            if self.goal_pose is None:
                raise ServiceException("goalPose 값이 비어있습니다", status_code=400)
            if len(self.goal_pose) < 3:
                raise ServiceException("goalPose 값이 x,y,z,rz 값을 가져야 합니다", status_code=400)
            if self.method is None:
                self.method = AmrMoveMethodEnum.PP.value
            if self.preset is None:
                self.preset = 0

        elif self.command == AmrMoveCommandEnum.MOVE_JOG:
            missing = [k for k in ("vx", "vy", "wz") if getattr(self, k) is None]
            if missing:
                raise ServiceException("vel 값이 비어있습니다", status_code=400)

        elif self.command in (AmrMoveCommandEnum.MOVE_STOP, AmrMoveCommandEnum.MOVE_PAUSE, AmrMoveCommandEnum.MOVE_RESUME):
            # 추가 요구 사항 없으면 패스
            pass

        elif self.command == AmrMoveCommandEnum.MOVE_X_LINEAR:
            if self.target is None:
                raise ServiceException("target 값이 비어있습니다", status_code=400)
            if self.speed is None:
                raise ServiceException("speed 값이 비어있습니다", status_code=400)

        elif self.command == AmrMoveCommandEnum.MOVE_CIRCULAR:
            if self.target is None:
                raise ServiceException("target 값이 비어있습니다", status_code=400)
            if self.speed is None:
                raise ServiceException("speed 값이 비어있습니다", status_code=400)
            if self.direction not in ("right", "left"):
                raise ServiceException("direction 값이 없거나 올바르지 않습니다. (right, left)", status_code=400)

        elif self.command == AmrMoveCommandEnum.MOVE_ROTATE:
            if self.target is None:
                raise ServiceException("target 값이 비어있습니다", status_code=400)
            if self.speed is None:
                raise ServiceException("speed 값이 비어있습니다", status_code=400)

        else:
            raise ServiceException(f"알 수 없는 command 값입니다. ({self.command})", status_code=400)

    def to_dict(self) -> dict[str, Any]:
        """
        - return: dict[str, Any]
        """
        d = {}
        d["id"] = self.id
        d["command"] = self.command
        d["status"] = self.status
        d["createdAt"] = self.created_at
        d["updateAt"] = self.update_at
        d["result"] = self.result
        d["message"] = self.message
        if self.command == AmrMoveCommandEnum.MOVE_GOAL:
            d["method"] = self.method
            d["preset"] = self.preset
            d["goalId"] = self.goal_id
        elif self.command == AmrMoveCommandEnum.MOVE_TARGET:
            d["method"] = self.method
            d["preset"] = self.preset
            d["goalPose"] = self.goal_pose
        elif self.command == AmrMoveCommandEnum.MOVE_JOG:
            d["vx"] = self.vx
            d["vy"] = self.vy
            d["wz"] = self.wz
        elif self.command == AmrMoveCommandEnum.MOVE_STOP or self.command == AmrMoveCommandEnum.MOVE_PAUSE or self.command == AmrMoveCommandEnum.MOVE_RESUME:
            pass
        elif self.command == AmrMoveCommandEnum.MOVE_X_LINEAR:
            d["target"] = self.target
            d["speed"] = self.speed
        elif self.command == AmrMoveCommandEnum.MOVE_CIRCULAR:
            d["direction"] = self.direction
            d["target"] = self.target
            d["speed"] = self.speed
        elif self.command == AmrMoveCommandEnum.MOVE_ROTATE:
            d["target"] = self.target
            d["speed"] = self.speed

        return d
