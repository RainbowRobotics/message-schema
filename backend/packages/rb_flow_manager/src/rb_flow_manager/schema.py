from enum import Enum


class RB_Flow_Manager_ProgramState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    WAITING = "waiting"
    STOPPED = "stopped"
    POST_START = "post_start"
    COMPLETED = "completed"
    ERROR = "error"
