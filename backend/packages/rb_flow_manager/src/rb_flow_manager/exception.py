"""Flow Manager 예외"""

class StopExecution(Exception):
    """실행 중단 예외"""

class BreakFolder(Exception):
    """현재 Folder(컨테이너) 내부 실행을 중단하고
    Folder 다음 스텝으로 넘어가고 싶을 때 사용."""
