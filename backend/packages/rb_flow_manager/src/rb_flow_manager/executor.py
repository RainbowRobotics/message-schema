import contextlib
import os
import queue
import signal
import sys
import time
from collections.abc import Callable, MutableMapping
from multiprocessing import Event, Manager, Process, Queue
from multiprocessing.managers import SyncManager
from multiprocessing.synchronize import Event as EventType
from threading import Thread
from typing import Any

from .context import ExecutionContext
from .controller.base_controller import BaseController
from .exception import StopExecution
from .schema import RB_Flow_Manager_ProgramState
from .step import Step


def create_global_resolver(controller, robot_model):
    """최상위 정의 — spawn/fork 모두 안전"""

    def _resolver(var_name: str):
        if (
            controller is not None
            and hasattr(controller, "get_global_variable")
            and callable(controller.get_global_variable)
        ):
            return controller.get_global_variable(robot_model, var_name)
        return None

    return _resolver


def _execute_tree_in_process(
    process_id: str,
    step: Step,
    state_dict: dict,
    result_queue: Queue,
    completion_event: EventType,
    pause_event: EventType,
    resume_event: EventType,
    stop_event: EventType,
    repeat_count: int = 1,
):
    """프로세스에서 실행될 트리 실행 함수"""
    ctx: ExecutionContext | None = None

    try:

        state_dict["state"] = RB_Flow_Manager_ProgramState.RUNNING
        state_dict["current_step_id"] = step.step_id
        state_dict["current_step_name"] = step.name
        state_dict["total_repeat"] = "infinity" if repeat_count == 0 else repeat_count
        state_dict["current_repeat"] = 0

        ctx = ExecutionContext(
            process_id,
            state_dict,
            result_queue=result_queue,
            pause_event=pause_event,
            resume_event=resume_event,
            stop_event=stop_event,
        )

        print(f"Executing tree in process: {process_id}")
        print(f"Step: {step.name}")
        print(f"Repeat count: {repeat_count}")

        if repeat_count > 0:
            for i in range(repeat_count):
                state_dict["current_repeat"] = i + 1

                ctx.check_stop()

                step.execute(ctx)
        else:
            i = 0
            while True:
                state_dict["current_repeat"] = i + 1

                ctx.check_stop()
                step.execute(ctx)
                i += 1

        if state_dict["state"] != RB_Flow_Manager_ProgramState.STOPPED:
            state_dict["state"] = RB_Flow_Manager_ProgramState.COMPLETED

    except StopExecution:
        state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED

        if ctx is not None:
            ctx.emit_stop(step.step_id)

    except RuntimeError as e:  # noqa: BLE001
        state_dict["state"] = RB_Flow_Manager_ProgramState.ERROR
        state_dict["error"] = str(e)

        if ctx is not None:
            ctx.emit_error(step.step_id, RuntimeError(state_dict["error"]))
    finally:
        # 완료 이벤트 설정
        completion_event.set()

        if ctx is not None:
            ctx.close()

        print(f"Execution completed: {process_id}")


class ScriptExecutor:
    """멀티프로세스 스크립트 실행 관리자"""

    def __init__(
        self,
        *,
        # zenoh_client: ZenohClient,
        on_init: Callable[[dict[str, MutableMapping[str, Any]]], None] | None = None,
        on_start: Callable[[str], None] | None = None,
        on_pause: Callable[[str, str], None] | None = None,
        on_resume: Callable[[str, str], None] | None = None,
        on_stop: Callable[[str, str], None] | None = None,
        on_next: Callable[[str, str], None] | None = None,
        on_error: Callable[[str, str, Exception], None] | None = None,
        on_close: Callable[[], None] | None = None,
        on_done: Callable[[str], None] | None = None,
        on_complete: Callable[[str], None] | None = None,
        on_all_complete: Callable[[], None] | None = None,
        on_all_stop: Callable[[], None] | None = None,
        on_all_pause: Callable[[], None] | None = None,
        controller: BaseController | None = None,
    ):
        self._on_init = on_init
        self._on_start = on_start
        self._on_pause = on_pause
        self._on_resume = on_resume
        self._on_stop = on_stop
        self._on_next = on_next
        self._on_error = on_error
        self._on_close = on_close
        self._on_done = on_done
        self._on_complete = on_complete
        self._on_all_complete = on_all_complete
        self._on_all_stop = on_all_stop
        self._on_all_pause = on_all_pause

        self.controller = controller

        self.manager: SyncManager | None = None
        self.processes: dict[str, Process] = {}
        self.state_dicts: dict[str, MutableMapping[str, Any]] = {}
        self.completion_events: dict[str, EventType] = {}
        self.result_queue: Queue = Queue()
        self._monitor_thread: Thread | None = None
        self._stop_monitor: EventType = Event()
        self.pause_events: dict[str, EventType] = {}
        self.resume_events: dict[str, EventType] = {}
        self.stop_events: dict[str, EventType] = {}

        # self._zenoh_client = zenoh_client

    def start(
        self,
        process_id: str,
        step: Step,
        repeat_count: int = 1,
        *,
        robot_model: str | None = None,
        category: str | None = None,
    ) -> bool:
        """스크립트 실행 시작"""
        if process_id in self.processes and self.processes[process_id].is_alive():
            print(f"Process {process_id} is already running")
            return False

        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._stop_monitor.clear()  # 혹시라도 눌려 있을 수 있으니 초기화
            self._start_monitor()

        if self._on_init is not None:
            self._on_init(self.state_dicts)

        if self.controller is not None:
            self.controller.on_init(self.state_dicts)

        self._ensure_manager()

        if self.manager is None:
            raise RuntimeError("Manager is not initialized")

        # 이벤트와 상태 딕셔너리 생성
        self.state_dicts[process_id] = self.manager.dict()
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.IDLE
        self.state_dicts[process_id]["current_step_id"] = None
        self.state_dicts[process_id]["robot_model"] = robot_model
        self.state_dicts[process_id]["category"] = category
        self.completion_events[process_id] = Event()

        self.pause_events[process_id] = Event()
        self.resume_events[process_id] = Event()
        self.stop_events[process_id] = Event()

        # 프로세스 생성 및 시작
        process = Process(
            target=_execute_tree_in_process,
            args=(
                process_id,
                step,
                self.state_dicts[process_id],
                self.result_queue,
                self.completion_events[process_id],
                self.pause_events[process_id],
                self.resume_events[process_id],
                self.stop_events[process_id],
                repeat_count,
            ),
        )

        process.start()

        if self._on_start is not None:
            self._on_start(process_id)

        if self.controller is not None:
            self.controller.on_start(process_id)

        self.processes[process_id] = process

        print(f"Started process: {process_id}")
        return True

    def _drain_events(self):
        while True:
            try:
                evt = self.result_queue.get_nowait()
            except queue.Empty:
                break

            evt_type = evt.get("type")
            pid = evt.get("process_id")
            step_id = evt.get("step_id")

            try:

                if evt_type == "next":
                    if self._on_next is not None:
                        self._on_next(pid, step_id)

                    if self.controller is not None:
                        self.controller.on_next(pid, step_id)

                elif evt_type == "done":
                    if self._on_done is not None:
                        self._on_done(pid)

                    if self.controller is not None:
                        self.controller.on_done(pid, step_id)

                elif evt_type == "error":
                    err_repr = evt.get("error")
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.ERROR

                    if self._on_error is not None:
                        self._on_error(pid, step_id, RuntimeError(err_repr))

                    if self.controller is not None:
                        self.controller.on_error(pid, step_id, RuntimeError(err_repr))

                    self.stop_all()
                    break

                elif evt_type == "pause":
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.PAUSED

                    if self._on_pause is not None:
                        self._on_pause(pid, step_id)

                    if self.controller is not None:
                        self.controller.on_pause(pid, step_id)

                elif evt_type == "resume":
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.RUNNING

                    if self._on_resume is not None:
                        self._on_resume(pid, step_id)

                    if self.controller is not None:
                        self.controller.on_resume(pid, step_id)

                elif evt_type == "stop":
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.STOPPED

                    if self._on_stop is not None:
                        self._on_stop(pid, step_id)

                    if self.controller is not None:
                        self.controller.on_stop(pid, step_id)

            except Exception as e:
                print(f"Error draining events: {e}")
                self.result_queue.put(
                    {"type": "error", "process_id": pid, "step_id": step_id, "error": str(e)}
                )
                raise e

    def _start_monitor(self):
        """백그라운드 모니터 스레드 시작"""
        self._monitor_thread = Thread(target=self._monitor_processes, daemon=True)
        self._monitor_thread.start()

    def _monitor_processes(self):
        """프로세스 완료 감시 → 자원 정리"""
        while not self._stop_monitor.is_set():
            if not self.processes:
                # 아직 아무 것도 없으면 살짝 쉰다
                time.sleep(0.2)
                continue

            self._drain_events()

            finished = []
            for pid, ev in list(self.completion_events.items()):
                if ev.wait(timeout=0.1):
                    finished.append(pid)

            for pid in finished:
                if self._on_complete is not None:
                    self._on_complete(pid)

                if self.controller is not None:
                    self.controller.on_complete(pid)

                p = self.processes.pop(pid, None)
                if p:
                    p.join(timeout=0.5)
                self.completion_events.pop(pid, None)
                self.state_dicts.pop(pid, None)
                self.pause_events.pop(pid, None)
                self.resume_events.pop(pid, None)
                self.stop_events.pop(pid, None)

            if not self.processes:
                if self._on_all_complete is not None:
                    self._on_all_complete()

                if self.controller is not None:
                    self.controller.on_all_complete()

                self._auto_cleanup()
                break

    def _auto_cleanup(self):
        """모든 스크립트가 끝났을 때만 실행되는 정리"""
        print("\n=== All processes completed, auto cleanup ===", flush=True)
        if self.manager is not None:
            with contextlib.suppress(Exception):
                self.manager.shutdown()
            self.manager = None

        if self._on_close is not None:
            self._on_close()

        if self.controller is not None:
            self.controller.on_close()

    def _ensure_manager(self):
        if self.manager is None:
            self.manager = Manager()

    def _safe_shutdown_manager(self):
        """Manager를 안전하게 종료"""
        try:
            if self.manager is not None:
                self.manager.shutdown()
            print("Manager shutdown completed")
        except RuntimeError as e:
            print(f"Manager shutdown warning: {e}")
        finally:
            self.manager = None

    def _cleanup_finished_processes(self):
        """종료된 프로세스들만 정리"""
        for process_id in list(self.processes.keys()):
            p = self.processes[process_id]
            if not p.is_alive():
                p.join(timeout=1)

                # 프로세스/이벤트/상태 일괄 정리
                self.processes.pop(process_id, None)
                self.completion_events.pop(process_id, None)
                self.state_dicts.pop(process_id, None)

                # 이벤트(일시정지/재개/정지)도 함께 제거해 누수 방지
                self.pause_events.pop(process_id, None)
                self.resume_events.pop(process_id, None)
                self.stop_events.pop(process_id, None)

        # 모든 프로세스가 정리되면 manager도 종료
        if not self.processes:
            self._safe_shutdown_manager()

    def pause(self, process_id: str) -> bool:
        """스크립트 일시정지"""
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        self.pause_events[process_id].set()
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.PAUSED

        step_id = self.state_dicts[process_id]["current_step_id"]

        if self._on_pause is not None:
            self._on_pause(process_id, step_id)

        if self.controller is not None:
            self.controller.on_pause(process_id, step_id)

        print(f"Paused process: {process_id}", flush=True)

        if all(
            self.state_dicts[pid]["state"] == RB_Flow_Manager_ProgramState.PAUSED
            for pid in self.processes
        ):
            if self._on_all_pause is not None:
                self._on_all_pause()

            if self.controller is not None:
                self.controller.on_all_pause()

            print("All processes paused", flush=True)

        return True

    def resume(self, process_id: str) -> bool:
        """스크립트 재개"""
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        self.resume_events[process_id].set()

        step_id = self.state_dicts[process_id]["current_step_id"]
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.RUNNING

        if self._on_resume is not None:
            self._on_resume(process_id, step_id)

        if self.controller is not None:
            self.controller.on_resume(process_id, step_id)

        print(f"Resumed process: {process_id}", flush=True)

        return True

    def stop(self, process_id: str) -> bool:
        """스크립트 중지"""
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        self.stop_events[process_id].set()

        step_id = self.state_dicts[process_id]["current_step_id"]
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.STOPPED

        if self._on_stop is not None:
            self._on_stop(process_id, step_id)

        if self.controller is not None:
            self.controller.on_stop(process_id, step_id)

        if all(
            self.state_dicts[pid]["state"] == RB_Flow_Manager_ProgramState.STOPPED
            for pid in self.processes
        ):
            if self._on_all_stop is not None:
                self._on_all_stop()

            if self.controller is not None:
                self.controller.on_all_stop()

            print("All processes stopped", flush=True)

        completed = self._wait_completion(process_id, timeout=2.0)

        if completed:
            print(f"Stopped softly process: {process_id}")
            return True

        with contextlib.suppress(Exception):
            self.processes[process_id].terminate()

        completed = self._wait_completion(process_id, timeout=3.0)

        if completed or not self.processes[process_id].is_alive():
            print(f"Stopped terminate process: {process_id}")
            return True

        try:
            if hasattr(self.processes[process_id], "kill"):
                self.processes[process_id].kill()
            elif sys.platform != "win32":
                os.kill(self.processes[process_id].pid or 0, signal.SIGKILL)

        except RuntimeError as e:
            print(f"Stopped kill process: {process_id} error: {e}")

        self.processes[process_id].join(timeout=1)

        if process_id in self.completion_events:
            self.completion_events[process_id].set()

        print(f"Stopped kill process: {process_id}")

        return True

    def _wait_completion(self, process_id: str, timeout: float) -> bool:
        ev = self.completion_events.get(process_id)
        if not ev:
            return False
        return ev.wait(timeout=timeout)

    def wait_all(self, timeout: float | None = None) -> bool:
        """모든 프로세스가 완료될 때까지 대기

        Args:
            timeout: 최대 대기 시간(초). None이면 무제한 대기

        Returns:
            모든 프로세스가 정상 완료되면 True, 타임아웃되면 False
        """
        start_time = time.time()

        while self.processes:
            if timeout and (time.time() - start_time) > timeout:
                return False

            all_done = all(not p.is_alive() for p in self.processes.values())
            if all_done:
                # 모니터 스레드가 정리할 시간을 줌
                time.sleep(0.5)
                return True

            time.sleep(0.1)

        return True

    def get_state(self, process_id: str) -> dict:
        """프로세스 상태 조회"""
        if process_id not in self.state_dicts:
            return {"state": RB_Flow_Manager_ProgramState.IDLE}

        state_dict = dict(self.state_dicts[process_id])
        state_dict["is_alive"] = (
            self.processes[process_id].is_alive() if process_id in self.processes else False
        )
        return state_dict

    def get_all_states(self) -> dict[str, dict]:
        """모든 프로세스 상태 조회"""
        return {pid: self.get_state(pid) for pid in self.processes}

    def stop_all(self):
        """모든 스크립트 중지"""
        for process_id in list(self.processes.keys()):
            self.stop(process_id)

        if self._on_all_stop is not None:
            self._on_all_stop()

        if self.controller is not None:
            self.controller.on_all_stop()

        self._auto_cleanup()

    def pause_all(self):
        """모든 스크립트 일시정지"""
        for process_id in list(self.processes.keys()):
            self.pause(process_id)

        if self._on_all_pause is not None:
            self._on_all_pause()

        if self.controller is not None:
            self.controller.on_all_pause()
