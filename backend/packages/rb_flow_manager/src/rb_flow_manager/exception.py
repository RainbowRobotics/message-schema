"""Flow Manager 예외"""

class StopExecution(Exception):
    """실행 중단 예외"""

class BreakFolder(Exception):
    """현재 Folder(컨테이너) 내부 실행을 중단하고
    Folder 다음 스텝으로 넘어가고 싶을 때 사용."""

class BreakRepeat(Exception):
    """가장 가까운 RepeatStep만 탈출"""

class JumpToStepException(Exception):
    """특정 step_id로 점프"""

    def __init__(self, target_step_id: str):
        super().__init__(f"jump to step {target_step_id}")
        self.target_step_id = target_step_id
