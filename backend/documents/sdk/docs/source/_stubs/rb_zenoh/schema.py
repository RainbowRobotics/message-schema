from dataclasses import dataclass
from enum import Enum


class OverflowPolicy(Enum):
    DROP_NEW = "drop_new"
    DROP_OLDEST = "drop_oldest"
    LATEST_ONLY = "latest_only"


@dataclass
class SubscribeOptions:
    dispatch: str = "immediate"
    sample_every: int = 1
    rate_limit_per_sec: int | None = None
    maxsize: int | None = None
    mem_budget_mb: float = 32.0
    ema_alpha: float = 0.25
    safety: float = 0.85
    overflow: OverflowPolicy = OverflowPolicy.DROP_OLDEST
    expected_avg_bytes: int | None = None
    allowed_same_sender: bool = False
    done_func: object | None = None


class CallbackEntry:
    pass
