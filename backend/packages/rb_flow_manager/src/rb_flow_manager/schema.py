from enum import Enum
from typing import TYPE_CHECKING, TypedDict, Union

if TYPE_CHECKING:
    from .step import Step


class SyncState(TypedDict):
    arrived: int        # 현재 phase에 도착한 수
    phase: int          # barrier 세대
    parties_cur: int    # 현재 phase에서 기다려야 할 인원
    parties_next: int   # 다음 phase부터 적용될 인원(동적 join/leave 대비)

class MakeProcessArgs(TypedDict):
    process_id: str
    step: "Step"
    repeat_count: int = 1
    robot_model: str | None = None
    category: str | None
    step_mode: bool = False
    min_step_interval: float | None = None
    is_ui_execution: bool = False
    post_tree:  Union["Step", None] = None

class RB_Flow_Manager_ProgramState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    WAITING = "waiting"
    STOPPED = "stopped"
    POST_START = "post_start"
    COMPLETED = "completed"
    ERROR = "error"
