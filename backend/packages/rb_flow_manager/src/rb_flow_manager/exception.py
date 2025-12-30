"""Flow Manager 예외"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .step import Step

class StopExecution(Exception):
    """실행 중단 예외"""

    def __init__(self, message: str):
        super().__init__(message)
        self.message = message

class BreakFolder(Exception):
    """현재 Folder(컨테이너) 내부 실행을 중단하고
    Folder 다음 스텝으로 넘어가고 싶을 때 사용."""

class BreakRepeat(Exception):
    """가장 가까운 RepeatStep만 탈출"""

class ContinueRepeat(Exception):
    """다음 내용 무시하고 RepeatStep 다음 부분부터 계속"""

class JumpToStepException(Exception):
    """특정 step_id로 점프"""

    def __init__(self, target_step_id: str):
        super().__init__(f"jump to step {target_step_id}")
        self.target_step_id = target_step_id

class ChangeSubTaskException(Exception):
    """서브 태스크 변경 예외"""

    def __init__(self, task_id: str, sub_task_tree: Step, sub_task_post_tree: Step | None = None):
        super().__init__(f"SubTask Change Exception: task_id={task_id}")
        self.task_id = task_id
        self.sub_task_tree = sub_task_tree
        self.sub_task_post_tree = sub_task_post_tree

class SubTaskHaltException(Exception):
    """서브 태스크 실행 중단 예외"""
