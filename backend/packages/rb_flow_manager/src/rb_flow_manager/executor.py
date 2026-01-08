import contextlib
import multiprocessing as mp
import os
import queue
import time
from collections.abc import Callable, MutableMapping
from multiprocessing import Queue
from multiprocessing.context import SpawnProcess
from multiprocessing.managers import SyncManager
from multiprocessing.synchronize import Event as EventType
from threading import Lock, Thread
from typing import Any, Literal

from .context import ExecutionContext
from .controller.base_controller import BaseController
from .exception import (
    BreakRepeat,
    ChangeSubTaskException,
    ContinueRepeat,
    JumpToStepException,
    StopExecution,
    SubTaskHaltException,
)
from .schema import RB_Flow_Manager_ProgramState
from .step import Step

# zenoh_client = ZenohClient()
# ZenohManager.set_shared_client(zenoh_client)

POLLING_INTERVAL = 0.1
EVENT_BATCH_INTERVAL = 0.05

def _execute_tree_in_process(
    process_id: str,
    step: Step,
    post_tree: Step | None,
    state_dict: dict,
    result_queue: Queue,
    completion_event: EventType,
    pause_event: EventType,
    resume_event: EventType,
    stop_event: EventType,
    repeat_count: int = 1,
    min_step_interval: float | None = None,
    step_mode: bool = False,
):
    """
    프로세스에서 실행될 트리 실행 함수

    - step(메인 트리)을 repeat_count 만큼 실행 (0이면 무한 반복)
    - 실행 중 Jump/Stop/SubTask 변경 등의 예외를 처리
    - 마지막에 post_tree가 있으면 후처리 트리 실행
    - 종료 시 completion_event.set() 보장
    """
    os.environ["_PYINSTALLER_WORKER"] = "1"

    ctx: ExecutionContext | None = None
    error_msg: str | None = None

    # post_tree가 SubTaskChange로 바뀔 수 있으므로 mutable 래퍼로 감싼다
    post_tree_ref: dict[str, Step | None] = {"value": post_tree}

    def set_error(msg: str):
        nonlocal error_msg
        state_dict["state"] = RB_Flow_Manager_ProgramState.ERROR
        state_dict["error"] = msg
        error_msg = msg
        print(f"Execution error: {msg}", flush=True)

    def emit_error_or_stop(step_id: str, msg: str):
        # "Execution stopped by user" 문자열 기반 정책은 기존 그대로 유지
        if ctx is None:
            return
        if "Execution stopped by user" in msg:
            ctx.emit_stop(step_id)
        else:
            ctx.emit_error(step_id, RuntimeError(msg))

    def init_state():
        state_dict["state"] = RB_Flow_Manager_ProgramState.RUNNING
        state_dict["step"] = step.to_dict()
        state_dict["current_step_id"] = step.step_id
        state_dict["current_step_name"] = step.name
        state_dict["total_repeat"] = "infinity" if repeat_count == 0 else repeat_count
        state_dict["sub_task_list"] = []
        state_dict["current_repeat"] = 0
        state_dict.pop("error", None)

    def execute_task(tree: Step, current_repeat: int, *, is_sub_task: bool = False):
        """
        step 트리 1회 실행.
        - JumpToStep / SubTaskChange 같은 제어 예외를 내부에서 처리(또는 재귀)
        """
        if not is_sub_task:
            state_dict["condition_map"] = {}

        state_dict["current_repeat"] = current_repeat

        try:
            # 기존 정책 유지:
            # - 2회차 이상이면 root는 다시 실행하지 않고 children만 실행
            if state_dict["current_repeat"] > 1:
                tree.execute_children(ctx)
            else:
                tree.execute(ctx)

        except JumpToStepException as e:
            # 특정 step으로 점프해서 children 실행
            tree.execute_children(ctx, target_step_id=e.target_step_id)
            return

        except ChangeSubTaskException as e:
            # sub-task 트리로 교체 실행 (재귀)
            if e.sub_task_tree is None:
                raise RuntimeError("SubTask: Tree is not found.") from e

            if e.sub_task_post_tree is not None:
                post_tree_ref["value"] = e.sub_task_post_tree

            sub_task_tree = e.sub_task_tree

            try:
                state_dict["sub_task_list"] = [{"task_id": sub_task_tree.step_id, "sub_task_type": "CHANGE"}]
                ctx.emit_sub_task_start(sub_task_tree.step_id, "CHANGE")

                execute_task(e.sub_task_tree, current_repeat, is_sub_task=True)
            except SubTaskHaltException:
                pass
            finally:
                state_dict["sub_task_list"] = []
                ctx.emit_sub_task_done(sub_task_tree.step_id, "CHANGE")

            return

    def run_main_loop():
        if repeat_count > 0:
            for i in range(1, repeat_count + 1):
                execute_task(step, i)
        else:
            i = 1
            while True:
                execute_task(step, i)
                i += 1

    def run_post_tree():
        pt = post_tree_ref["value"]
        if pt is None:
            return

        stop_event.clear()
        state_dict["ignore_stop"] = True

        if ctx is not None:
            ctx.emit_post_start()

        pt.execute(ctx)

    try:
        init_state()

        ctx = ExecutionContext(
            process_id,
            state_dict,
            result_queue=result_queue,
            pause_event=pause_event,
            resume_event=resume_event,
            stop_event=stop_event,
            min_step_interval=min_step_interval,
            step_mode=step_mode,
        )

        # ---- 메인 실행 ----
        try:
            run_main_loop()
            if state_dict["state"] != RB_Flow_Manager_ProgramState.STOPPED:
                state_dict["state"] = RB_Flow_Manager_ProgramState.COMPLETED

        except StopExecution:
            # post_tree가 없을 때만 여기서 STOPPED 처리(기존 정책 유지)
            if post_tree_ref["value"] is None:
                state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
                if ctx is not None:
                    ctx.emit_stop(step.step_id)

        except RuntimeError as e:
            # post_tree가 없을 때만 여기서 ERROR 처리(기존 정책 유지)
            if post_tree_ref["value"] is None:
                set_error(str(e))
                emit_error_or_stop(step.step_id, str(e))

        except JumpToStepException:
            # 메인 루프 바깥으로 점프 예외가 올라오는 경우는 무시(기존 정책 유지)
            pass

        # ---- post_tree 실행(항상 finally에 가까운 의미) ----
        try:
            run_post_tree()

        except BreakRepeat:
            pass

        except ContinueRepeat:
            pass

        except StopExecution:
            state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
            if ctx is not None:
                ctx.emit_stop(step.step_id)

        except RuntimeError as e:
            set_error(str(e))
            emit_error_or_stop(step.step_id, str(e))

        except JumpToStepException as e:
            raise RuntimeError("JumpToStep: Post program tree is not allowed.") from e

    finally:
        print("completion_event set >>>>>>>", completion_event.is_set(), flush=True)
        completion_event.set()

        if ctx is not None:
            ctx.close()


class ScriptExecutor:
    """멀티프로세스 스크립트 실행 관리자"""

    def __init__(
        self,
        *,
        # zenoh_client: ZenohClient,
        on_init: Callable[[dict[str, MutableMapping[str, Any]]], None] | None = None,
        on_start: Callable[[str], None] | None = None,
        on_pause: Callable[[str, str], None] | None = None,
        on_wait: Callable[[str, str], None] | None = None,
        on_resume: Callable[[str, str], None] | None = None,
        on_stop: Callable[[str, str], None] | None = None,
        on_next: Callable[[str, str], None] | None = None,
        on_sub_task_start: Callable[[str, Literal["INSERT", "CHANGE"]], None] | None = None,
        on_sub_task_done: Callable[[str, Literal["INSERT", "CHANGE"]], None] | None = None,
        on_error: Callable[[str, str, Exception], None] | None = None,
        on_close: Callable[[], None] | None = None,
        on_done: Callable[[str], None] | None = None,
        on_post_start: Callable[[str], None] | None = None,
        on_complete: Callable[[str], None] | None = None,
        on_all_complete: Callable[[], None] | None = None,
        on_all_stop: Callable[[], None] | None = None,
        on_all_pause: Callable[[], None] | None = None,
        controller: BaseController | None = None,
        min_step_interval: float | None = 0.01,
    ):
        self._on_init = on_init
        self._on_start = on_start
        self._on_pause = on_pause
        self._on_wait = on_wait
        self._on_resume = on_resume
        self._on_stop = on_stop
        self._on_next = on_next
        self._on_sub_task_start = on_sub_task_start
        self._on_sub_task_done = on_sub_task_done
        self._on_error = on_error
        self._on_close = on_close
        self._on_done = on_done
        self._on_post_start = on_post_start
        self._on_complete = on_complete
        self._on_all_complete = on_all_complete
        self._on_all_stop = on_all_stop
        self._on_all_pause = on_all_pause

        self._mp_ctx = mp.get_context("spawn")

        self.controller = controller

        self._run_generation = 0
        self._pid_generation: dict[str, int] = {}
        self.min_step_interval: float | None = min_step_interval
        self.manager: SyncManager | None = None
        self.processes: dict[str, SpawnProcess] = {}
        self.state_dicts: dict[str, MutableMapping[str, Any]] = {}
        self.completion_events: dict[str, EventType] = {}
        self.result_queues: dict[str, Queue] = {}
        self._monitor_thread: Thread | None = None
        self._stop_monitor: EventType = self._mp_ctx.Event()
        self.pause_events: dict[str, EventType] = {}
        self.resume_events: dict[str, EventType] = {}
        self.stop_events: dict[str, EventType] = {}
        self._mu = Lock()

        # self._zenoh_client = zenoh_client

    def start(
        self,
        process_id: str,
        step: Step,
        repeat_count: int = 1,
        *,
        robot_model: str | None = None,
        category: str | None = None,
        step_mode: bool = False,
        min_step_interval: float | None = None,
        is_ui_execution: bool = False,
        post_tree: Step | None = None,
    ) -> bool:
        """스크립트 실행 시작"""
        if process_id in self.processes and self.processes[process_id].is_alive():
            print(f"Process {process_id} is already running")

            if step_mode:
                if self.pause_events[process_id].is_set():
                    self.resume(process_id)
                else:
                    self.pause(process_id)

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

        if self._on_start is not None:
            self._on_start(process_id)

        if self.controller is not None:
            self.controller.on_start(process_id)

        self._run_generation += 1
        gen = self._run_generation

        if min_step_interval is not None:
            self.min_step_interval = min_step_interval

        # 이벤트와 상태 딕셔너리 생성
        self.state_dicts[process_id] = self.manager.dict()
        self.result_queues[process_id] = self._mp_ctx.Queue(maxsize=10)
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.IDLE
        self.state_dicts[process_id]["current_step_id"] = None
        self.state_dicts[process_id]["robot_model"] = robot_model
        self.state_dicts[process_id]["category"] = category
        self.state_dicts[process_id]["generation"] = gen
        self.state_dicts[process_id]["step_mode"] = step_mode
        self.state_dicts[process_id]["is_ui_execution"] = is_ui_execution

        self._pid_generation[process_id] = gen

        self.completion_events[process_id] = self._mp_ctx.Event()

        self.pause_events[process_id] = self._mp_ctx.Event()
        self.resume_events[process_id] = self._mp_ctx.Event()
        self.stop_events[process_id] = self._mp_ctx.Event()

        # 프로세스 생성 및 시작
        process = self._mp_ctx.Process(
            target=_execute_tree_in_process,
            args=(
                process_id,
                step,
                post_tree,
                self.state_dicts[process_id],
                self.result_queues[process_id],
                self.completion_events[process_id],
                self.pause_events[process_id],
                self.resume_events[process_id],
                self.stop_events[process_id],
                repeat_count,
                self.min_step_interval,
                step_mode,
            ),
        )

        process.start()

        self.processes[process_id] = process

        return True

    def _drain_events(self, process_id: str):
        while True:
            if self.manager is None:
                break

            if process_id not in self.result_queues:
                break

            result_queue = self.result_queues[process_id]

            try:
                evt = result_queue.get_nowait()
            except queue.Empty:
                break

            evt_type = evt.get("type")
            pid = evt.get("process_id")
            step_id = evt.get("step_id")
            is_wait = evt.get("is_wait", False)
            evt_gen = evt.get("generation")
            sub_task_id = evt.get("sub_task_id")
            sub_task_type = evt.get("sub_task_type")

            cur_gen = self._pid_generation.get(pid)

            if cur_gen is None or evt_gen != cur_gen:
                print(f"ignore stale event: {evt}", flush=True)
                continue

            if self.state_dicts.get(pid) is None:
                print(f"ignore process not found: {pid}", flush=True)
                continue

            try:

                if evt_type == "next":
                    if self._on_next is not None:
                        self._on_next(pid, step_id)

                    if self.controller is not None:
                        self.controller.on_next(pid, step_id)

                elif evt_type == "sub_task_start":
                    if self._on_sub_task_start is not None:
                        self._on_sub_task_start(pid, sub_task_id, sub_task_type)

                    if self.controller is not None:
                        self.controller.on_sub_task_start(pid, sub_task_id, sub_task_type)

                elif evt_type == "sub_task_done":
                    if self._on_sub_task_done is not None:
                        self._on_sub_task_done(pid, sub_task_type)

                    if self.controller is not None:
                        self.controller.on_sub_task_done(pid, sub_task_id, sub_task_type)

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
                        self.controller.on_error(pid, step_id, err_repr)

                    time.sleep(0.5)

                    self.stop_all()
                    break

                elif evt_type == "post_start":
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.POST_START

                    if self._on_post_start is not None:
                        self._on_post_start(pid)

                    if self.controller is not None:
                        self.controller.on_post_start(pid)

                elif evt_type == "pause":
                    self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.WAITING if is_wait else RB_Flow_Manager_ProgramState.PAUSED

                    if is_wait:
                        if self._on_wait is not None:
                            self._on_wait(pid, step_id)

                        if self.controller is not None:
                            self.controller.on_wait(pid, step_id)
                    else:
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
                raise e

    def _start_monitor(self):
        """백그라운드 모니터 스레드 시작"""
        self._monitor_thread = Thread(target=self._monitor_processes, daemon=True)
        self._monitor_thread.start()

    def _monitor_processes(self):
        """프로세스 완료 감시 → 자원 정리"""
        last_event_process_time = time.time()

        while not self._stop_monitor.is_set():
            if not self.processes:
                # 아직 아무 것도 없으면 살짝 쉰다
                time.sleep(0.02)
                continue

            current_time = time.time()

            if current_time - last_event_process_time >= EVENT_BATCH_INTERVAL:
                for pid in list(self.processes.keys()):
                    self._drain_events(pid)
                last_event_process_time = current_time

            finished: list[str] = []

            for pid, ev in list(self.completion_events.items()):
                p = self.processes.get(pid)

                if ev.is_set() or (p is not None and not p.is_alive()):
                    finished.append(pid)

            for pid in finished:
                self._drain_events(pid)

            for pid in finished:
                if self._on_complete is not None:
                    self._on_complete(pid)

                if self.controller is not None:
                    self.controller.on_complete(pid)

                with self._mu:
                    p = self.processes.pop(pid, None)
                    if p:
                        p.join(timeout=0.1)
                    self.completion_events.pop(pid, None)
                    self.state_dicts.pop(pid, None)
                    self.pause_events.pop(pid, None)
                    self.resume_events.pop(pid, None)
                    self.stop_events.pop(pid, None)
                    self._pid_generation.pop(pid, None)

            if not self.processes:
                if self._on_all_complete is not None:
                    self._on_all_complete()

                if self.controller is not None:
                    self.controller.on_all_complete()

                self._auto_cleanup()
                break

            time.sleep(POLLING_INTERVAL)

    def _auto_cleanup(self):
        """모든 스크립트가 끝났을 때만 실행되는 정리"""
        print("\n=== All processes completed, auto cleanup ===", flush=True)

        if self.manager is not None:
            with contextlib.suppress(Exception):
                self.manager.shutdown()
            self.manager = None

        self._stop_monitor.set()

        with contextlib.suppress(Exception):
            for pid in list(self.result_queues.keys()):
                self.result_queues[pid].close()

        self.result_queues = {}

        # (선택) 모니터 스레드 핸들 무효화
        self._monitor_thread = None

        if self._on_close is not None:
            self._on_close()

        if self.controller is not None:
            self.controller.on_close()

    def _ensure_manager(self):
        if self.manager is None:
            self.manager = self._mp_ctx.Manager()

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

        if self.pause_events[process_id].is_set():
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

        if self.resume_events[process_id].is_set():
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
        self.pause_events[process_id].clear()
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        if self.stop_events[process_id].is_set():
            return False

        step_id = self.state_dicts[process_id]["current_step_id"]
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.STOPPED

        self.stop_events[process_id].set()

        if self._on_stop is not None and step_id is not None:
            self._on_stop(process_id, step_id)

        if self.controller is not None and step_id is not None:
            self.controller.on_stop(process_id, step_id)

        # if all(
        #     self.state_dicts[pid]["state"] == RB_Flow_Manager_ProgramState.STOPPED
        #     for pid in self.processes
        # ):
        if self._on_all_stop is not None:
            self._on_all_stop()

        if self.controller is not None:
            self.controller.on_all_stop()

        print("All processes stopped", flush=True)

        # completed = self._wait_completion(process_id, timeout=0.3)

        # if completed:
        #     print(f"Stopped softly process: {process_id}")
        #     return True

        # with contextlib.suppress(Exception):
        #     self.processes[process_id].terminate()

        # completed = self._wait_completion(process_id, timeout=0.7)

        # if completed or not self.processes[process_id].is_alive():
        #     print(f"Stopped terminate process: {process_id}", flush=True)
        #     return True

        # try:
        #     if hasattr(self.processes[process_id], "kill"):
        #         self.processes[process_id].kill()
        #     elif sys.platform != "win32":
        #         os.kill(self.processes[process_id].pid or 0, signal.SIGKILL)

        # except RuntimeError as e:
        #     print(f"Stopped kill process: {process_id} error: {e}", flush=True)

        # self.processes[process_id].join(timeout=1)

        # if process_id in self.completion_events:
        #     self.completion_events[process_id].set()

        # print(f"Stopped kill process: {process_id}", flush=True)

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
        with self._mu:
            if process_id not in self.state_dicts:
                return {"state": RB_Flow_Manager_ProgramState.IDLE}

            state_dict = dict(self.state_dicts[process_id])
            is_alive = self.processes.get(process_id)
            state_dict["is_alive"] = is_alive.is_alive() if is_alive else False
            return state_dict

    def get_all_states(self) -> dict[str, dict]:
        """모든 프로세스 상태 조회"""
        with self._mu:
            pids = list(self.processes.keys())
        return {pid: self.get_state(pid) for pid in pids}

    def stop_all(self):
        """모든 스크립트 중지"""
        for process_id in list(self.processes.keys()):
            self.stop(process_id)

        if self._on_all_stop is not None:
            self._on_all_stop()

        if self.controller is not None:
            self.controller.on_all_stop()

        # self._auto_cleanup()

    def pause_all(self):
        """모든 스크립트 일시정지"""
        for process_id in list(self.processes.keys()):
            self.pause(process_id)

        if self._on_all_pause is not None:
            self._on_all_pause()

        if self.controller is not None:
            self.controller.on_all_pause()

    def resume_all(self):
        """모든 스크립트 재개"""
        for process_id in list(self.processes.keys()):
            self.resume(process_id)
