from abc import (
    ABC,
    abstractmethod,
)
from collections.abc import (
    MutableMapping,
)
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
        """강제 중지 시점에 호출"""

    @abstractmethod
    def on_pause(self, process_id: str, step_id: str) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_resume(self, process_id: str, step_id: str) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_next(self, process_id: str, step_id: str) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_error(self, process_id: str, step_id: str, error: Exception) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_complete(self, process_id: str, step_id: str) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_process_complete(self, process_id: str) -> None:
        """프로세스 완료 시점에 호출"""

    @abstractmethod
    def on_close(self) -> None:
        """컨트롤러 종료"""

    @abstractmethod
    def on_all_complete(self) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_all_stop(self) -> None:
        """각 Step에서 실제 동작을 수행"""

    @abstractmethod
    def on_all_pause(self) -> None:
        """각 Step에서 실제 동작을 수행"""
