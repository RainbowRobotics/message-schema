from abc import ABC, abstractmethod
from collections.abc import MutableMapping
from typing import Any


class BaseController(ABC):
    """모든 컨트롤러가 따라야 하는 공통 인터페이스"""

    @abstractmethod
    def on_init(self, state_dicts: dict[str, MutableMapping[str, Any]]) -> None:
        """컨트롤러 초기화"""

    @abstractmethod
    def on_start(self, process_id: str) -> None:
        """스크립트/플로우 시작 시점에 한 번 호출"""

    @abstractmethod
    def on_stop(self, process_id: str, step_id: str) -> None:
        """각 Step이 Stop 될때 동작을 수행"""

    @abstractmethod
    def on_pause(self, process_id: str, step_id: str) -> None:
        """각 Step이 Pause 될때 실제 동작을 수행"""

    @abstractmethod
    def on_resume(self, process_id: str, step_id: str) -> None:
        """각 Step이 Resume 될때 실제 동작을 수행"""

    @abstractmethod
    def on_next(self, process_id: str, step_id: str) -> None:
        """각 Step이 다음 Step으로 이동 할때 동작을 수행"""

    @abstractmethod
    def on_done(self, process_id: str, step_id: str) -> None:
        """각 Step이 완료 되었을때 동작을 수행"""

    @abstractmethod
    def on_error(self, process_id: str, step_id: str, error: Exception) -> None:
        """각 Step에서 Error 발생 시 동작을 수행"""

    @abstractmethod
    def on_complete(self, process_id: str) -> None:
        """스크립트/플로우 완료 시점에 동작을 수행"""

    @abstractmethod
    def on_close(self) -> None:
        """컨트롤러 종료"""

    @abstractmethod
    def on_all_complete(self) -> None:
        """모든 스크립트/플로우 완료 시점에 동작을 수행"""

    @abstractmethod
    def on_all_stop(self) -> None:
        """모든 스크립트/플로우 Stop 시점에 동작을 수행"""

    @abstractmethod
    def on_all_pause(self) -> None:
        """모든 스크립트/플로우 Pause 시점에 동작을 수행"""
