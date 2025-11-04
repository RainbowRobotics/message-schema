import builtins
import contextlib
import sys
import time
from collections.abc import Callable
from multiprocessing import Queue
from multiprocessing.synchronize import Event as EventType
from typing import Any

from rb_sdk.manipulate import RBManipulateSDK

from .exception import StopExecution
from .schema import RB_Flow_Manager_ProgramState


class ExecutionContext:
    """스크립트 실행 컨텍스트"""

    def __init__(
        self,
        process_id: str,
        state_dict: dict,
        result_queue: Queue,
        pause_event: EventType,
        resume_event: EventType,
        stop_event: EventType,
    ):
        self.process_id = process_id
        self.state_dict = state_dict
        self.pause_event = pause_event
        self.resume_event = resume_event
        self.stop_event = stop_event
        self.result_queue = result_queue
        self.variables: dict[str, Any] = {}
        self.sdk_functions: dict[str, Callable] = {}
        self.data: dict[str, Any] = {}  # 사용자 정의 데이터 저장소

        self._make_zenoh_sdk_method_key_value_map()

    def update_variables(self, variables: dict[str, Any]):
        self.variables.update(variables)

    def _store_variables(self):
        # TODO: zenoh publish or queryable variables
        pass

    def _make_zenoh_sdk_method_key_value_map(self):
        sdk_list = [{"name": "rb_manupulate_sdk", "class": RBManipulateSDK}]

        for item in sdk_list:
            sdk = item["class"]()

            public_methods = {
                f"{item['name']}.{name}": getattr(sdk, name)
                for name in dir(sdk)
                if callable(getattr(sdk, name)) and not name.startswith("_")
            }

            self.sdk_functions.update(public_methods)

    def pause(self):
        """현재 스크립트를 일시정지"""
        self.pause_event.set()

        self.emit_pause(self.state_dict["current_step_id"])

        self._wait_for_resume()

    def stop(self):
        """현재 스크립트를 중지"""
        self.stop_event.set()

        self.emit_stop(self.state_dict["current_step_id"])

        self._wait_for_resume()

    def _wait_for_resume(self):
        """외부 pause 신호를 즉시 반영하는 세이프포인트"""
        if self.stop_event.is_set():
            self.state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
            raise StopExecution("Execution stopped by user")

        if self.pause_event.is_set():
            self.state_dict["state"] = RB_Flow_Manager_ProgramState.PAUSED

            # resume 올 때까지 대기
            while True:
                if self.stop_event.is_set():
                    self.state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
                    raise StopExecution("Execution stopped by user")
                if self.resume_event.wait(timeout=0.01):  # 10ms 타임슬라이스
                    self.resume_event.clear()
                    self.pause_event.clear()
                    self.state_dict["state"] = RB_Flow_Manager_ProgramState.RUNNING
                    break

    def check_stop(self):
        """stop/pause/resume 명령 처리"""
        self._wait_for_resume()  # 즉시 멈춤/중지 반영

    def emit_next(self, step_id: str):
        """next 이벤트 발생"""

        self.result_queue.put(
            {
                "type": "next",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": None,
            }
        )

    def emit_pause(self, step_id: str):
        """pause 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "pause",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": None,
            }
        )

    def emit_resume(self, step_id: str):
        """resume 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "resume",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": None,
            }
        )

    def emit_stop(self, step_id: str):
        """stop 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "stop",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": None,
            }
        )

    def emit_error(self, step_id: str, error: Exception):
        """error 이벤트 발생"""

        self.result_queue.put(
            {
                "type": "error",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": str(error),
            }
        )

    def emit_complete(self, step_id: str):
        """complete 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "complete",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "error": None,
            }
        )

    def _safe_close_sdk(self):
        """SDK 안전 종료"""
        with contextlib.suppress(Exception):
            if self.sdk_functions is not None:
                for key in list(self.sdk_functions.keys()):
                    if key.endswith(".close"):
                        self.sdk_functions[key]()

    def close(self):
        """안전하게 close"""
        self._store_variables()
        self._safe_close_sdk()

    def __del__(self):
        """소멸자"""
        if getattr(sys, "is_finalizing", lambda: False)():
            return

        with contextlib.suppress(builtins.BaseException):
            self.close()
