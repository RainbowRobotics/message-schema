from dataclasses import dataclass, field


@dataclass
class ControlCache:
    state: str = "RUNNING"
    last_ms: float = 0.0


@dataclass
class Locator:
    task_id: str
    sync_task_ids: list[str] = field(default_factory=list)
    program_id: str
    node_path: list[str]
    offset: int
