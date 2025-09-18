import asyncio
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Literal


class OverflowPolicy(Enum):
    DROP_NEW = "drop_new"
    DROP_OLDEST = "drop_oldest"
    LATEST_ONLY = "latest_only"


DispatchMode = Literal["immediate", "queue"]


@dataclass
class _BatchOptions:
    ms: int | None = None
    max: int = 256


@dataclass
class SubscribeOptions:
    dispatch: DispatchMode = "immediate"

    sample_every: int = 1
    # 초당 콜백 호출 상한
    rate_limit_per_sec: int | None = None

    maxsize: int | None = None  # 0/None → 자동용량(EMA+메모리 예산)
    mem_budget_mb: float = 32.0
    ema_alpha: float = 0.25
    safety: float = 0.85
    overflow: OverflowPolicy = OverflowPolicy.DROP_OLDEST
    batch_opts: _BatchOptions = field(default_factory=_BatchOptions)
    # 예상 평균 바이트(EMA 초기 힌트). 없으면 첫 메시지 크기에서 시작
    expected_avg_bytes: int | None = None


class CallbackEntry:
    def __init__(self, topic: str, callback, opts: SubscribeOptions):
        self.topic = topic
        self.callback = callback
        self.opts = opts
        self.q: asyncio.Queue = asyncio.Queue(maxsize=0)  # queue 모드에서만 사용
        self.metrics: dict[str, Any] = {
            "ema_bytes": None,
            "capacity": opts.maxsize,
            "delivered": 0,
            "dropped_new": 0,
            "dropped_oldest": 0,
            "dropped_latest": 0,
            "last_qsize": 0,
            "last_ts": None,
            "_sample_i": 0,
        }
