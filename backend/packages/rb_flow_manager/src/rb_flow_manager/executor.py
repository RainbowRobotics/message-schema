import contextlib
import multiprocessing as mp
import os
import queue
import time
from collections.abc import Callable, MutableMapping
from multiprocessing import Queue
from multiprocessing.context import SpawnProcess
from multiprocessing.managers import (
    AcquirerProxy,
    ConditionProxy,
    DictProxy,
    SyncManager,
)
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
from .schema import MakeProcessArgs, RB_Flow_Manager_ProgramState
from .step import Step

# 상수 정의
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
    start_event: EventType | None = None,
    repeat_count: int = 1,
    min_step_interval: float | None = None,
    step_mode: bool = False,
    *,
    sync_members: dict[str, list[str]],
    sync_state: dict[str, dict[str, int]],
    sync_cond: ConditionProxy,
    sync_lock: AcquirerProxy,
):
    """
    프로세스에서 실행될 트리 실행 함수

    - step(메인 트리)을 repeat_count 만큼 실행 (0이면 무한 반복)
    - 실행 중 Jump/Stop/SubTask 변경 등의 예외를 처리
    - 마지막에 post_tree가 있으면 후처리 트리 실행
    - 종료 시 completion_event.set() 보장
    """
    t0 = time.monotonic()
    print(f"[child {process_id}] entered in {time.monotonic()-t0:.6f}", flush=True)
    os.environ["_PYINSTALLER_WORKER"] = "1"

    # Ready 신호 전송
    result_queue.put({
        "type": "ready",
        "process_id": process_id,
        "ts": time.time(),
        "generation": state_dict.get("generation"),
    })
    print(f"[child {process_id}] ready signal sent in {time.monotonic()-t0:.6f}", flush=True)

    # Start 이벤트 대기
    if start_event is not None:
        start_event.wait()
    print(f"[child {process_id}] start_event.wait() in {time.monotonic()-t0:.6f}", flush=True)

    ctx: ExecutionContext | None = None
    error_msg: str | None = None
    post_tree_ref: dict[str, Step | None] = {"value": post_tree}

    def set_error(msg: str):
        nonlocal error_msg
        state_dict["state"] = RB_Flow_Manager_ProgramState.ERROR
        state_dict["error"] = msg
        error_msg = msg
        print(f"Execution error: {msg}", flush=True)

    def emit_error_or_stop(step_id: str, msg: str):
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
        """step 트리 1회 실행"""
        if not is_sub_task:
            state_dict["condition_map"] = {}

        state_dict["current_repeat"] = current_repeat

        try:
            # 2회차 이상이면 root는 다시 실행하지 않고 children만 실행
            if state_dict["current_repeat"] > 1:
                tree.execute_children(ctx)
            else:
                tree.execute(ctx)

        except JumpToStepException as e:
            tree.execute_children(ctx, target_step_id=e.target_step_id)
            return

        except ChangeSubTaskException as e:
            if e.sub_task_tree is None:
                raise RuntimeError("SubTask: Tree is not found.") from e

            if e.sub_task_post_tree is not None:
                post_tree_ref["value"] = e.sub_task_post_tree
            else:
                post_tree_ref["value"] = None

            sub_task_tree = e.sub_task_tree

            try:
                state_dict["sub_task_list"] = [
                    {"task_id": sub_task_tree.step_id, "sub_task_type": "CHANGE"}
                ]
                ctx.emit_sub_task_start(sub_task_tree.step_id, "CHANGE")
                execute_task(e.sub_task_tree, current_repeat, is_sub_task=True)
            except SubTaskHaltException:
                pass
            finally:
                if post_tree_ref["value"] is None:
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

        if len(state_dict["sub_task_list"]) > 0:
            sub_task_id = state_dict["sub_task_list"][-1]["task_id"]
            sub_task_type = state_dict["sub_task_list"][-1]["sub_task_type"]
            state_dict["sub_task_list"] = []
            ctx.emit_sub_task_done(sub_task_id, sub_task_type)

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

        # Sync 레지스트리 설정 (sync_parties 제거)
        ctx.sync_members = sync_members
        ctx.sync_state = sync_state
        ctx.sync_cond = sync_cond
        ctx.sync_lock = sync_lock

        # 메인 실행
        try:
            run_main_loop()
            if state_dict["state"] != RB_Flow_Manager_ProgramState.STOPPED:
                state_dict["state"] = RB_Flow_Manager_ProgramState.COMPLETED

        except StopExecution:
            if post_tree_ref["value"] is None:
                state_dict["state"] = RB_Flow_Manager_ProgramState.STOPPED
                if ctx is not None:
                    ctx.emit_stop(step.step_id)

        except RuntimeError as e:
            if post_tree_ref["value"] is None:
                set_error(str(e))
                emit_error_or_stop(step.step_id, str(e))

        except JumpToStepException:
            # 메인 루프 바깥으로 점프 예외가 올라오는 경우는 무시
            pass

        # post_tree 실행
        try:
            run_post_tree()

        except (BreakRepeat, ContinueRepeat):
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
        print(f"completion_event set >>> {completion_event.is_set()}", flush=True)
        completion_event.set()

        if ctx is not None:
            ctx.close()


class ScriptExecutor:
    """멀티프로세스 스크립트 실행 관리자"""

    def __init__(
        self,
        *,
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
        min_step_interval: float | None = 0,
    ):
        # 콜백 함수들
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

        # 멀티프로세싱 관련
        self._mp_ctx = mp.get_context("spawn")
        self.manager: SyncManager | None = None

        # 컨트롤러
        self.controller = controller

        # 실행 상태 관리
        self._run_generation = 0
        self._pid_generation: dict[str, int] = {}
        self.min_step_interval: float | None = min_step_interval

        # 프로세스 및 이벤트 관리
        self.processes: dict[str, SpawnProcess] = {}
        self.state_dicts: dict[str, MutableMapping[str, Any]] = {}
        self.completion_events: dict[str, EventType] = {}
        self.result_queues: dict[str, Queue] = {}
        self.pause_events: dict[str, EventType] = {}
        self.resume_events: dict[str, EventType] = {}
        self.stop_events: dict[str, EventType] = {}
        self.start_events: dict[str, EventType | None] = {}

        # 배치 실행 관리
        self._batch_ready_mu = Lock()
        self._batch_ready_count = 0
        self._batch_expected = 0
        self._batch_start_event: EventType | None = None

        # 동기화 관리 - _ensure_manager() 호출 전에 먼저 None으로 초기화
        self._sync_lock: AcquirerProxy | None = None
        self._sync_cond: ConditionProxy | None = None
        self._sync_state: DictProxy | None = None
        self._sync_members: DictProxy | None = None

        # 모니터 스레드
        self._monitor_thread: Thread | None = None
        self._stop_monitor: EventType = self._mp_ctx.Event()

        # 전역 락
        self._mu = Lock()

        # Manager 초기화 (모든 속성 초기화 후 호출)
        self._ensure_manager()

    def _ensure_manager(self):
        """Manager 인스턴스 생성 및 동기화 객체 초기화"""
        if self.manager is None:
            self.manager = self._mp_ctx.Manager()

        # Sync 레지스트리 초기화 (한 번만, sync_parties 제거)
        if self._sync_lock is None:
            self._sync_lock = self.manager.RLock()
            self._sync_cond = self.manager.Condition(self._sync_lock)
            self._sync_state = self.manager.dict()
            self._sync_members = self.manager.dict()

    def _collect_sync_flags(self, step: Step | None) -> set[str]:
        """Step 트리에서 모든 SyncStep의 flag 수집"""
        if step is None:
            return set()

        flags: set[str] = set()

        def walk(s: Step):
            if s.__class__.__name__ == "SyncStep":
                f = getattr(s, "flag", None)
                if f:
                    flags.add(str(f))

            for c in getattr(s, "children", []) or []:
                walk(c)

        walk(step)
        return flags

    def _register_batch(self, batch: list[MakeProcessArgs]) -> dict[str, set[str]]:
        """배치 실행을 위한 동기화 플래그 등록"""
        if self.manager is None:
            raise RuntimeError("Manager not initialized")

        pid_flags: dict[str, set[str]] = {}

        # pid별 flags 수집
        for arg in batch:
            pid = arg["process_id"]
            flags = self._collect_sync_flags(arg["step"]) | self._collect_sync_flags(
                arg.get("post_tree")
            )
            pid_flags[pid] = flags

        # 각 flag별로 이번 배치의 참가자 수 계산
        flag_counts: dict[str, int] = {}
        for _, flags in pid_flags.items():
            for flag in flags:
                flag_counts[flag] = flag_counts.get(flag, 0) + 1

        # sync_state 업데이트
        with self._sync_lock:
            for flag, _ in flag_counts.items():
                # members - 일반 list로 저장 (proxy 중첩 금지)
                members = list(self._sync_members.get(flag, []))

                # 이번 배치의 pid들만 추가
                for pid in pid_flags.items():
                    if flag in pid_flags[pid] and pid not in members:
                        members.append(pid)

                self._sync_members[flag] = members

                # state - parties는 누적이 아니라 현재 멤버 수로 설정
                st = dict(self._sync_state.get(flag, {}))
                if not st:
                    st = {
                        "arrived": 0,
                        "phase": 0,
                        "parties_cur": 0,
                        "parties_next": 0,
                    }

                # parties는 현재 멤버 수로 설정 (누적 X)
                st["parties_cur"] = len(members)
                st["parties_next"] = len(members)

                self._sync_state[flag] = st

        return pid_flags

    def make_process(
        self,
        args: MakeProcessArgs,
        *,
        multi_start: bool = False,
        start_event: EventType | None = None,
    ) -> SpawnProcess | None:
        """개별 프로세스 생성"""
        process_id = args.get("process_id")
        step = args.get("step")
        repeat_count = args.get("repeat_count")
        robot_model = args.get("robot_model")
        category = args.get("category")
        step_mode = args.get("step_mode")
        min_step_interval = args.get("min_step_interval")
        is_ui_execution = args.get("is_ui_execution")
        post_tree = args.get("post_tree")

        # 이미 실행 중인 프로세스 체크
        if process_id in self.processes and self.processes[process_id].is_alive():
            print(f"Process {process_id} is already running", flush=True)

            # state_dict 존재 확인
            if process_id not in self.state_dicts:
                print(f"[WARNING] Process {process_id} running but no state_dict", flush=True)
                return None

            # 실행 중인 프로세스가 step_mode면 토글
            is_step_mode = self.state_dicts[process_id].get("step_mode", False)
            print(f"[DEBUG] process_id={process_id}, step_mode={is_step_mode}", flush=True)

            if is_step_mode:
                current_state = self.state_dicts[process_id].get("state")
                print(f"[DEBUG] current_state={current_state}", flush=True)

                if current_state == RB_Flow_Manager_ProgramState.PAUSED:
                    print(f"[step_mode] Resuming {process_id}", flush=True)
                    self.resume(process_id)
                elif current_state == RB_Flow_Manager_ProgramState.RUNNING:
                    print(f"[step_mode] Pausing {process_id}", flush=True)
                    self.pause(process_id)
                elif current_state == RB_Flow_Manager_ProgramState.WAITING:
                    print(f"[step_mode] Resuming from WAITING {process_id}", flush=True)
                    self.resume(process_id)
                else:
                    print(f"[step_mode] Cannot toggle - state: {current_state}", flush=True)
            else:
                print(f"[INFO] Process {process_id} is not in step_mode", flush=True)

            return None

        # 모니터 스레드 시작
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._stop_monitor.clear()
            self._start_monitor()

        self._ensure_manager()

        if self.manager is None:
            raise RuntimeError("Manager is not initialized")

        # 세대(generation) 증가
        self._run_generation += 1
        gen = self._run_generation

        if min_step_interval is not None:
            self.min_step_interval = min_step_interval

        self._pid_generation[process_id] = gen

        # 상태 딕셔너리 생성 (start()에서 이미 생성했으면 재사용)
        if process_id not in self.state_dicts:
            self.state_dicts[process_id] = self.manager.dict()

        self.result_queues[process_id] = self._mp_ctx.Queue(maxsize=10)

        # 상태 정보 업데이트
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.RUNNING
        self.state_dicts[process_id]["current_step_id"] = None
        self.state_dicts[process_id]["robot_model"] = robot_model
        self.state_dicts[process_id]["category"] = category
        self.state_dicts[process_id]["generation"] = gen
        self.state_dicts[process_id]["step_mode"] = step_mode
        self.state_dicts[process_id]["is_ui_execution"] = is_ui_execution

        # 콜백 호출
        if self._on_init is not None:
            self._on_init(self.state_dicts)

        if self.controller is not None:
            self.controller.on_init(self.state_dicts)

        if self._on_start is not None:
            self._on_start(process_id)

        if self.controller is not None:
            self.controller.on_start(process_id)

        # 이벤트 생성
        self.completion_events[process_id] = self._mp_ctx.Event()
        self.pause_events[process_id] = self._mp_ctx.Event()
        self.resume_events[process_id] = self._mp_ctx.Event()
        self.stop_events[process_id] = self._mp_ctx.Event()
        self.start_events[process_id] = start_event if multi_start else None

        # 프로세스 생성
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
                self.start_events[process_id],
                repeat_count,
                self.min_step_interval,
                step_mode,
            ),
            kwargs={
                "sync_members": self._sync_members,
                "sync_state": self._sync_state,
                "sync_cond": self._sync_cond,
                "sync_lock": self._sync_lock,
            },
        )

        return process

    def start(self, args: MakeProcessArgs | list[MakeProcessArgs]) -> bool:
        """프로세스 시작 (단일 또는 배치)"""
        batch: list[MakeProcessArgs] = args if isinstance(args, list) else [args]
        is_batch = len(batch) > 1

        self._ensure_manager()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._stop_monitor.clear()
            self._start_monitor()

        # 단일 프로세스면 즉시 실행 (step_mode 토글 가능)
        if not is_batch:
            arg = batch[0]
            p = self.make_process(arg, multi_start=False, start_event=None)
            if p:
                p.start()
                self.processes[arg["process_id"]] = p
            return True

        # 배치 모드
        pid_flags = self._register_batch(batch)

        # 배치 시작 이벤트
        batch_start_event = self._mp_ctx.Event()

        # 배치 ready 카운트 초기화
        self._batch_expected = len(batch)
        self._batch_ready_count = 0
        self._batch_start_event = batch_start_event

        processes: list[SpawnProcess] = []
        pids: list[str] = []

        # 프로세스 생성
        for arg in batch:
            pid = arg["process_id"]

            # state_dict 초기화를 여기서 먼저 수행
            if self.manager is None:
                raise RuntimeError("Manager not initialized")

            if pid not in self.state_dicts:
                self.state_dicts[pid] = self.manager.dict()

            self.state_dicts[pid]["sync_flags"] = list(pid_flags.get(pid, set()))

            p = self.make_process(
                arg,
                multi_start=True,
                start_event=batch_start_event,
            )
            if p:
                processes.append(p)
                pids.append(pid)

        # 프로세스 시작
        for i, p in enumerate(processes):
            p.start()
            self.processes[pids[i]] = p

        return True

    def _drain_events(self, process_id: str):
        """프로세스의 이벤트 큐에서 모든 이벤트 처리"""
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

            # 오래된 이벤트 무시
            if cur_gen is None or evt_gen != cur_gen:
                print(f"Ignoring stale event: {evt}", flush=True)
                continue

            if self.state_dicts.get(pid) is None:
                print(f"Process not found: {pid}", flush=True)
                continue

            # 이벤트 타입별 처리
            self._handle_event(evt_type, pid, step_id, is_wait, sub_task_id, sub_task_type, evt)

    def _handle_event(
        self,
        evt_type: str,
        pid: str,
        step_id: str | None,
        is_wait: bool,
        sub_task_id: str | None,
        sub_task_type: str | None,
        evt: dict,
    ):
        """개별 이벤트 처리"""
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

            elif evt_type == "post_start":
                self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.POST_START
                if self._on_post_start is not None:
                    self._on_post_start(pid)
                if self.controller is not None:
                    self.controller.on_post_start(pid)

            elif evt_type == "pause":
                state = (
                    RB_Flow_Manager_ProgramState.WAITING
                    if is_wait
                    else RB_Flow_Manager_ProgramState.PAUSED
                )
                self.state_dicts[pid]["state"] = state

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

            elif evt_type == "control":
                action = evt.get("action")
                if action == "pause_all":
                    self.pause_all()
                elif action == "resume_all":
                    self.resume_all()

            elif evt_type == "ready":
                with self._batch_ready_mu:
                    self._batch_ready_count += 1
                    if (
                        self._batch_ready_count >= self._batch_expected
                        and self._batch_start_event is not None
                    ):
                        self._batch_start_event.set()

        except Exception as e:
            print(f"Error handling event: {e}", flush=True)
            raise

    def _start_monitor(self):
        """백그라운드 모니터 스레드 시작"""
        self._monitor_thread = Thread(target=self._monitor_processes, daemon=True)
        self._monitor_thread.start()

    def _monitor_processes(self):
        """프로세스 완료 감시 및 자원 정리"""
        last_event_process_time = time.time()

        while not self._stop_monitor.is_set():
            if not self.processes:
                time.sleep(0.02)
                continue

            current_time = time.time()

            # 이벤트 배치 처리
            if current_time - last_event_process_time >= EVENT_BATCH_INTERVAL:
                for pid in list(self.processes.keys()):
                    self._drain_events(pid)
                last_event_process_time = current_time

            # 완료된 프로세스 찾기
            finished: list[str] = []
            for pid, ev in list(self.completion_events.items()):
                p = self.processes.get(pid)
                if ev.is_set() or (p is not None and not p.is_alive()):
                    finished.append(pid)

            # 완료된 프로세스의 남은 이벤트 처리
            for pid in finished:
                self._drain_events(pid)

            # 완료된 프로세스 정리
            for pid in finished:
                if self._on_complete is not None:
                    self._on_complete(pid)
                if self.controller is not None:
                    self.controller.on_complete(pid)

                self._cleanup_process(pid)

            # 모든 프로세스 완료 시
            if not self.processes:
                if self._on_all_complete is not None:
                    self._on_all_complete()
                if self.controller is not None:
                    self.controller.on_all_complete()

                self._auto_cleanup()
                break

            time.sleep(POLLING_INTERVAL)

    def _cleanup_process(self, process_id: str):
        """개별 프로세스 정리"""
        with self._mu:
            p = self.processes.pop(process_id, None)
            if p:
                p.join(timeout=0.1)

            self.completion_events.pop(process_id, None)
            self.state_dicts.pop(process_id, None)
            self.pause_events.pop(process_id, None)
            self.resume_events.pop(process_id, None)
            self.stop_events.pop(process_id, None)
            self.start_events.pop(process_id, None)
            self._pid_generation.pop(process_id, None)

            # 큐 정리
            if process_id in self.result_queues:
                with contextlib.suppress(Exception):
                    self.result_queues[process_id].close()
                self.result_queues.pop(process_id, None)

    def _auto_cleanup(self):
        """모든 프로세스 종료 시 자동 정리"""
        print("\n=== All processes completed, auto cleanup ===", flush=True)

        self._stop_monitor.set()

        # 큐 정리
        with contextlib.suppress(Exception):
            for pid in list(self.result_queues.keys()):
                self.result_queues[pid].close()

        self.result_queues = {}
        self._monitor_thread = None

        # 콜백 호출
        if self._on_close is not None:
            self._on_close()
        if self.controller is not None:
            self.controller.on_close()

    def pause(self, process_id: str) -> bool:
        """스크립트 일시정지"""
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        # 이미 일시정지 상태면 무시
        if self.state_dicts[process_id].get("state") == RB_Flow_Manager_ProgramState.PAUSED:
            return False

        # pulse 패턴: clear 후 set
        self.pause_events[process_id].clear()
        self.pause_events[process_id].set()
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.PAUSED

        step_id = self.state_dicts[process_id].get("current_step_id")

        if self._on_pause is not None:
            self._on_pause(process_id, step_id)
        if self.controller is not None:
            self.controller.on_pause(process_id, step_id)

        print(f"Paused process: {process_id}", flush=True)

        # 모든 프로세스가 일시정지되었는지 확인
        if all(
            self.state_dicts[pid].get("state") == RB_Flow_Manager_ProgramState.PAUSED
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

        # 이미 실행 중이면 무시
        current_state = self.state_dicts[process_id].get("state")
        if current_state == RB_Flow_Manager_ProgramState.RUNNING:
            return False

        # pulse 패턴: clear 후 set
        self.resume_events[process_id].clear()
        self.resume_events[process_id].set()

        step_id = self.state_dicts[process_id].get("current_step_id")
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.RUNNING

        if self._on_resume is not None:
            self._on_resume(process_id, step_id)
        if self.controller is not None:
            self.controller.on_resume(process_id, step_id)

        return True

    def stop(self, process_id: str) -> bool:
        """스크립트 중지"""
        if process_id not in self.processes:
            print(f"Process {process_id} not found")
            return False

        if not self.processes[process_id].is_alive():
            print(f"Process {process_id} is not running")
            return False

        if self.stop_events[process_id].is_set():
            return False

        self.pause_events[process_id].clear()

        step_id = self.state_dicts[process_id].get("current_step_id")
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.STOPPED

        self.stop_events[process_id].set()

        if self._on_stop is not None and step_id is not None:
            self._on_stop(process_id, step_id)
        if self.controller is not None and step_id is not None:
            self.controller.on_stop(process_id, step_id)

        if self._on_all_stop is not None:
            self._on_all_stop()
        if self.controller is not None:
            self.controller.on_all_stop()

        print("Process stopped", flush=True)

        return True

    def stop_all(self):
        """모든 스크립트 중지"""
        for process_id in list(self.processes.keys()):
            self.stop(process_id)

        if self._on_all_stop is not None:
            self._on_all_stop()
        if self.controller is not None:
            self.controller.on_all_stop()

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

    def wait_all(self, timeout: float | None = None) -> bool:
        """모든 프로세스가 완료될 때까지 대기"""
        start_time = time.time()

        while self.processes:
            if timeout and (time.time() - start_time) > timeout:
                return False

            all_done = all(not p.is_alive() for p in self.processes.values())
            if all_done:
                time.sleep(0.5)  # 모니터 스레드가 정리할 시간
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
