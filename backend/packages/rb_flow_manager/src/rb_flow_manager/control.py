import threading
from enum import Enum


class RB_Flow_Manager_ProgramState(str, Enum):
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    WAITING = "WAITING"
    ERROR = "ERROR"
    COMPLETED = "COMPLETED"


class ControlFlags:
    def __init__(self):
        self.state = RB_Flow_Manager_ProgramState.RUNNING

        self._paused = threading.Event()
        self._resume = threading.Event()
        self._stop = threading.Event()

        self._resume.set()

    def set_state(self, new_state: RB_Flow_Manager_ProgramState):
        self.state = new_state

        if (
            new_state == RB_Flow_Manager_ProgramState.PAUSED
            or new_state == RB_Flow_Manager_ProgramState.WAITING
        ):
            self._resume.clear()

            if new_state == RB_Flow_Manager_ProgramState.PAUSED:
                self._paused.set()

        elif new_state == RB_Flow_Manager_ProgramState.RUNNING:
            self._resume.set()
            self._paused.clear()
        elif new_state == RB_Flow_Manager_ProgramState.STOPPED:
            self._stop.set()
            self._resume.set()  # 정지는 resume도 풀어야 함

    def wait_if_paused(self):
        self._resume.wait()

    def raise_if_stopped(self):
        if self._stop.is_set():
            raise StopIteration("Stopped by control")

    def is_paused(self):
        return self._paused.is_set()
