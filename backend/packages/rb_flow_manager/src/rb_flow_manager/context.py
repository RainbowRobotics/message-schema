import builtins
import contextlib
import sys
import time
from collections.abc import (
    Callable,
)
from multiprocessing import (
    Queue,
)
from multiprocessing.synchronize import (
    Event as EventType,
)
from typing import Any, Literal

from rb_sdk.base import RBBaseSDK
from rb_sdk.manipulate import RBManipulateSDK
from rb_utils.parser import t_to_dict

from .exception import BreakFolder, StopExecution, SubTaskHaltException
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
        *,
        min_step_interval: float | None = None,
        step_mode: bool = False,
    ):
        self.process_id = process_id
        self.state_dict = state_dict
        self.parent_process_id = state_dict.get("parent_process_id")
        self.pause_event = pause_event
        self.resume_event = resume_event
        self.stop_event = stop_event
        self.result_queue = result_queue
        self.variables: dict[str, dict[str, Any]] = {
            "local": {},
            "global": {},
        }
        self.sdk_functions: dict[str, Callable] = {}
        self._sdk_roots: dict[str, Any] | None = None
        self._arg_scope: list[dict[str, Any]] = []
        self._generation = state_dict.get("generation")
        self.min_step_interval = min_step_interval
        self._folder_depth = 0
        self.step_mode = step_mode
        self.step_num = 1
        self.current_depth = 0
        self._step_ticket = 1 if not step_mode else 0
        self.is_ui_execution = state_dict.get("is_ui_execution", False)
        self.data: dict[str, Any] = {}  # 사용자 정의 데이터 저장소

    def update_vars_to_state_dict(self):
        current = dict(self.state_dict.get("variables", {}))

        for k, v in self.variables.get("global", {}).items():
            current[k] = {"value": v, "type": "global"}

        for k, v in self.variables.get("local", {}).items():
            current[k] = {"value": v, "type": "local"}

        self.state_dict["variables"] = current

    def update_local_variables(self, variables: dict[str, Any]):
        self.variables["local"].update(t_to_dict(variables))
        self.update_vars_to_state_dict()

    def update_global_variables(self, variables: dict[str, Any]):
        self.variables["global"].update(variables)
        self.update_vars_to_state_dict()

    def get_global_variable(self, var_name: str) -> Any:
        robot_model = self.state_dict.get("robot_model", None)
        category = self.state_dict.get("category", None)

        if category is None or robot_model is None:
            return None

        try:
            if category == "manipulate":
                fn = self.get_sdk_function("rb_manipulate_sdk.get_data.get_variable")

                res = fn(robot_model, var_name) if fn is not None else None

                if res is None:
                    raise RuntimeError(f"Failed to get global variable: {var_name}")

                self.update_global_variables({
                    var_name: res
                })

                return res
        except Exception as e:
            raise e

    def _ensure_sdk_roots(self):
        if self._sdk_roots is not None:
            return

        self._sdk_roots = {
            "rb_base_sdk": RBBaseSDK(),
            "rb_manipulate_sdk": RBManipulateSDK(),
        }

    def get_sdk_function(self, func_name: str) -> Callable | None:
        if not func_name:
            return None

        cached = self.sdk_functions.get(func_name)
        if cached is not None:
            return cached

        self._ensure_sdk_roots()
        if self._sdk_roots is None:
            return None

        parts = func_name.split(".")
        if len(parts) < 2:
            return None

        obj = self._sdk_roots.get(parts[0])
        if obj is None:
            return None

        for part in parts[1:]:
            if part.startswith("_") or not hasattr(obj, part):
                return None
            obj = getattr(obj, part)

        if not callable(obj):
            return None

        self.sdk_functions[func_name] = obj
        return obj

    def step_barrier(self):
        """step_mode면, 이 스텝 수행 후 일시정지하고
        resume 신호 올 때까지 대기"""
        if not self.step_mode or self.step_num < 2:
            return

        if self.stop_event.is_set():
            self.state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
            raise StopExecution("Execution stopped by user")

        if not self.pause_event.is_set():
            self.pause(is_wait=True)

        self._wait_for_resume()

    def push_args(self, mapping: dict[str, Any] | None):
        self._arg_scope.append(mapping or {})

    def pop_args(self):
        if self._arg_scope:
            self._arg_scope.pop()

    def lookup(self, key: str) -> Any:
        idx = len(self._arg_scope) - 2

        while idx >= 0:
            scope = self._arg_scope[idx]
            if key in scope:
                return scope[key]
            idx -= 1
        raise KeyError(f"parent pointer not found: {key}")

    def pause(self, is_wait: bool = False):
        """현재 스크립트를 일시정지"""
        if self.pause_event.is_set():
            return

        self.pause_event.set()

        self.emit_pause(self.state_dict["current_step_id"], is_wait)

        self._wait_for_resume()

    def resume(self):
        """현재 스크립트를 재개"""
        if not self.resume_event.is_set():
            return

        self.resume_event.set()
        self.pause_event.clear()
        self.state_dict["state"] = RB_Flow_Manager_ProgramState.RUNNING

    def stop(self):
        """현재 스크립트를 중지"""
        if self.stop_event.is_set():
            return

        self.stop_event.set()
        self.pause_event.clear()

        self.emit_stop(self.state_dict["current_step_id"])

        self._wait_for_resume()

    def _wait_for_resume(self):
        """외부 pause 신호를 즉시 반영하는 세이프포인트"""
        if self.stop_event.is_set() and not self.state_dict.get("ignore_stop", False):
            self.state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
            raise StopExecution("Execution stopped by user")



        if self.pause_event.is_set():
            self.resume_event.clear()

            self.state_dict["state"] = RB_Flow_Manager_ProgramState.PAUSED

            # resume 올 때까지 대기
            while True:
                if self.stop_event.is_set() and not self.state_dict.get("ignore_stop", False):
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
                "generation": self._generation,
                "error": None,
            }
        )

    def emit_pause(self, step_id: str, is_wait: bool = False):
        """pause 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "pause",
                "process_id": self.process_id,
                "step_id": step_id,
                "is_wait": is_wait,
                "parent_process_id": self.parent_process_id,
                "ts": time.time(),
                "generation": self._generation,
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
                "parent_process_id": self.parent_process_id,
                "ts": time.time(),
                "generation": self._generation,
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
                "parent_process_id": self.parent_process_id,
                "ts": time.time(),
                "generation": self._generation,
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
                "generation": self._generation,
                "error": str(error),
            }
        )

    def emit_subtask_sync_register(self, sub_task_tree, post_tree=None, subtask_type: Literal["INSERT", "CHANGE"] = "INSERT"):
        """서브태스크 sync flags 등록 요청 이벤트"""
        self.result_queue.put({
            "type": "subtask_sync_register",
            "process_id": self.process_id,
            "subtask_type": subtask_type,
            "sub_task_tree": sub_task_tree.to_dict() if sub_task_tree else None,
            "post_tree": post_tree.to_dict() if post_tree else None,
            "ts": time.time(),
            "generation": self._generation,
        })

    def emit_subtask_sync_unregister(self, sub_task_tree, post_tree=None, subtask_type: Literal["INSERT", "CHANGE"] = "INSERT"):
        """서브태스크 sync flags 해제 요청 이벤트"""
        self.result_queue.put({
            "type": "subtask_sync_unregister",
            "process_id": self.process_id,
            "subtask_type": subtask_type,
            "sub_task_tree": sub_task_tree.to_dict() if sub_task_tree else None,
            "post_tree": post_tree.to_dict() if post_tree else None,
            "ts": time.time(),
            "generation": self._generation,
        })

    def emit_main_tree_sync_unregister(self):
        """메인 트리 sync flags 해제 요청 (CHANGE 타입 전환 시)"""
        self.result_queue.put({
            "type": "main_tree_sync_unregister",
            "process_id": self.process_id,
            "ts": time.time(),
            "generation": self._generation,
        })

    def emit_sub_task_start(self, task_id: str, sub_task_type: Literal["INSERT", "CHANGE"]):
        """sub_task_start 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "sub_task_start",
                "process_id": self.process_id,
                "sub_task_id": task_id,
                "sub_task_type": sub_task_type,
                "ts": time.time(),
                "generation": self._generation,
                "error": None,
            }
        )

    def emit_sub_task_done(self, task_id: str, sub_task_type: Literal["INSERT", "CHANGE"]):
        """sub_task_done 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "sub_task_done",
                "process_id": self.process_id,
                "sub_task_id": task_id,
                "sub_task_type": sub_task_type,
                "ts": time.time(),
                "generation": self._generation,
                "error": None,
            }
        )

    def emit_event_sub_task_start(
        self,
        *,
        event_task_id: str,
        event_tree,
        step_id: str | None = None,
        run_mode: Literal["SYNC", "ASYNC"] = "ASYNC",
        call_seq: int | None = None,
    ):
        """EventSubTask 실행 요청 이벤트"""
        print("emit_event_sub_task_start", flush=True)
        self.result_queue.put(
            {
                "type": "event_sub_task_start",
                "process_id": self.process_id,
                "event_task_id": event_task_id,
                "event_tree": event_tree.to_dict() if event_tree is not None else None,
                "step_id": step_id,
                "run_mode": run_mode,
                "call_seq": call_seq,
                "ts": time.time(),
                "generation": self._generation,
                "error": None,
            }
        )

    def emit_post_start(self):
        """post_start 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "post_start",
                "process_id": self.process_id,
                "ts": time.time(),
                "generation": self._generation,
                "error": None,
            }
        )

    def emit_done(self, step_id: str):
        """done 이벤트 발생"""
        self.result_queue.put(
            {
                "type": "done",
                "process_id": self.process_id,
                "step_id": step_id,
                "ts": time.time(),
                "generation": self._generation,
                "error": None,
            }
        )

    def pause_all(self):
        self.result_queue.put({
            "type": "control",
            "action": "pause_all",
            "process_id": self.process_id,
            "ts": time.time(),
            "generation": self._generation,
        })

    def resume_all(self):
        self.result_queue.put({
            "type": "control",
            "action": "resume_all",
            "process_id": self.process_id,
            "ts": time.time(),
            "generation": self._generation,
        })

    def enter_folder(self):
        """Folder 진입"""
        self._folder_depth += 1

    def leave_folder(self):
        """Folder 탈출"""
        self._folder_depth -= 1

    def break_folder(self):
        """Folder 실행 중단"""
        if self._folder_depth > 0:
            raise BreakFolder()

    def halt_sub_task(self):
        """서브 태스크 실행 중단"""
        raise SubTaskHaltException()

    def _safe_close_sdk(self):
        """SDK 안전 종료"""
        with contextlib.suppress(Exception):
            if self._sdk_roots is not None:
                for sdk in self._sdk_roots.values():
                    close_fn = getattr(sdk, "close", None)
                    if callable(close_fn):
                        close_fn()

    def close(self):
        """안전하게 close"""
        self._safe_close_sdk()

    def __del__(self):
        """소멸자"""
        if getattr(sys, "is_finalizing", lambda: False)():
            return

        with contextlib.suppress(builtins.BaseException):
            self.close()
