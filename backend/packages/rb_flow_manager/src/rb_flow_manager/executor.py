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
EVENT_BATCH_INTERVAL = 0.01


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
    process_alive: dict[str, bool] | None = None,
    parent_map: dict[str, str | None] | None = None,
    all_state_dicts: dict[str, MutableMapping[str, Any]] | None = None,
):
    """
    프로세스에서 실행될 트리 실행 함수

    - step(메인 트리)을 repeat_count 만큼 실행 (0이면 무한 반복)
    - 실행 중 Jump/Stop/SubTask 변경 등의 예외를 처리
    - 마지막에 post_tree가 있으면 후처리 트리 실행
    - 종료 시 completion_event.set() 보장
    """
    os.environ["_PYINSTALLER_WORKER"] = "1"

    # 프로세스 부팅 완료 신호(배치 start barrier 용)
    result_queue.put({
        "type": "ready",
        "process_id": process_id,
        "ts": time.time(),
        "generation": state_dict.get("generation"),
    })

    # Start 이벤트 대기
    if start_event is not None:
        start_event.wait()

    ctx: ExecutionContext | None = None
    post_tree_ref: dict[str, Step | None] = {"value": post_tree}

    def set_error(msg: str):
        state_dict["state"] = RB_Flow_Manager_ProgramState.ERROR
        state_dict["error"] = msg

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
            pending_target_step_id: str | None = None
            max_jump_hops = 1000
            jump_hops = 0

            while True:
                try:
                    # 2회차 이상이면 root는 다시 실행하지 않고 children만 실행
                    if pending_target_step_id is not None:
                        tree.execute_children(ctx, target_step_id=pending_target_step_id)
                        pending_target_step_id = None
                    elif state_dict["current_repeat"] > 1:
                        tree.execute_children(ctx)
                    else:
                        tree.execute(ctx)
                    break
                except JumpToStepException as e:
                    pending_target_step_id = e.target_step_id
                    jump_hops += 1
                    if jump_hops > max_jump_hops:
                        raise RuntimeError(
                            "JumpToStep: exceeded max jump hops in a single execution."
                        ) from e

        except ChangeSubTaskException as e:
            if e.sub_task_tree is None:
                raise RuntimeError("SubTask: Tree is not found.") from e

            # 서브태스크 flags 등록
            ctx.emit_subtask_sync_register(
                e.sub_task_tree,
                e.sub_task_post_tree,
                "CHANGE"
            )

            time.sleep(0.05)  # executor가 처리할 시간

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

        # parent process는 자신의 모든 자손 프로세스(parent_process_id 체인)가
        # 끝난 뒤 post_tree 실행
        #
        # 중요:
        # - all_state_dicts는 child process 관점에서 스냅샷일 수 있어 동적 spawn된
        #   event child를 놓칠 수 있다.
        # - 따라서 shared parent_map/process_alive를 우선 진실원천으로 사용한다.
        # - 또한 CallEventStep 직후 실제 child 등록 전 레이스를 막기 위해
        #   event_start_pending_map이 비어있는지도 함께 확인한다.
        if process_alive is not None and parent_map is not None:
            while True:
                pending_start_map = dict(state_dict.get("event_start_pending_map", {}))
                has_pending_event_start = len(pending_start_map) > 0
                has_alive_children = False
                child_pids: list[str] = []

                # descendants를 재귀적으로 수집 (shared parent_map 기준)
                queue_pids: list[str] = [process_id]
                visited: set[str] = set()
                while queue_pids:
                    parent = queue_pids.pop(0)
                    if parent in visited:
                        continue
                    visited.add(parent)
                    for pid in list(parent_map.keys()):
                        if pid == process_id:
                            continue
                        if parent_map.get(pid) == parent and pid not in child_pids:
                            child_pids.append(pid)
                            queue_pids.append(pid)

                # 보조 경로: 같은 세대에서 이미 보이는 state_dict 기반 자손도 union
                # (parent_map 갱신 타이밍 레이스 완화용)
                if all_state_dicts is not None:
                    queue_pids = [process_id]
                    visited.clear()
                    while queue_pids:
                        parent = queue_pids.pop(0)
                        if parent in visited:
                            continue
                        visited.add(parent)
                        for pid, child_sd in all_state_dicts.items():
                            if pid == process_id:
                                continue
                            if (child_sd or {}).get("parent_process_id") == parent and pid not in child_pids:
                                child_pids.append(pid)
                                queue_pids.append(pid)

                for pid in child_pids:
                    child_alive = bool(process_alive.get(pid, False))
                    if child_alive:
                        has_alive_children = True
                        break

                # 비동기 event spawn ack 대기 중이거나,
                # 실제 자손 프로세스가 살아있으면 post_tree 진입 금지
                if not has_pending_event_start and not has_alive_children:
                    break

                time.sleep(0.05)

        stop_event.clear()
        # post_tree는 정책상 stop 이후에도 실행되므로, 누적된 pause/resume latch에 걸려 진입 직후 block되지 않도록 강제로 정리
        pause_event.clear()
        resume_event.clear()
        state_dict["ignore_stop"] = True
        state_dict["current_step_id"] = pt.step_id
        state_dict["current_step_name"] = pt.name

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
            shared_state_dicts=all_state_dicts,
        )

        # Sync 레지스트리 설정
        ctx.sync_members = sync_members
        ctx.sync_state = sync_state
        ctx.sync_cond = sync_cond
        ctx.sync_lock = sync_lock

        # 실제 실행 시작 가능 시점 신호(CallEventStep 대기 해제용)
        result_queue.put({
            "type": "exec_ready",
            "process_id": process_id,
            "ts": time.time(),
            "generation": state_dict.get("generation"),
        })

        # 메인 실행
        try:
            run_main_loop()
            if state_dict["state"] != RB_Flow_Manager_ProgramState.STOPPED:
                state_dict["state"] = RB_Flow_Manager_ProgramState.COMPLETED

        except StopExecution:
            # stop 요청 상태로 마킹 (post_tree는 정책상 실행 가능)
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

        # post_tree 실행 (정책상 stop 이후에도 실행 가능)
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
        if process_alive is not None:
            process_alive[process_id] = False

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
        on_sub_task_start: Callable[[str, str, Literal["INSERT", "CHANGE"]], None] | None = None,
        on_sub_task_done: Callable[[str, str, Literal["INSERT", "CHANGE"]], None] | None = None,
        on_error: Callable[[str, str, Exception], None] | None = None,
        on_close: Callable[[], None] | None = None,
        on_done: Callable[[str, str], None] | None = None,
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
        self._sync_lock = None
        self._sync_cond = None
        self._sync_state: DictProxy | None = None
        self._sync_members: DictProxy | None = None
        self._process_alive: DictProxy | None = None
        self._parent_map: DictProxy | None = None

        # 모니터 스레드
        self._monitor_thread: Thread | None = None
        self._stop_monitor: EventType = self._mp_ctx.Event()
        self._pending_complete_callbacks: set[str] = set()
        self._event_wait_parent_by_child: dict[str, str] = {}
        self._event_wait_children_count: dict[str, int] = {}
        self._event_wait_resume_pending: dict[str, int] = {}
        self._event_start_seq_by_child: dict[str, int] = {}

        # 전역 락
        self._mu = Lock()

        # Manager 초기화 (모든 속성 초기화 후 호출)
        self._ensure_manager()

    def _ensure_manager(self):
        """Manager 인스턴스 생성 및 동기화 객체 초기화"""
        if self.manager is None:
            self.manager = self._mp_ctx.Manager()

        # dict류만 manager로
        if self._sync_state is None:
            self._sync_state = self.manager.dict()
        if self._sync_members is None:
            self._sync_members = self.manager.dict()
        if self._process_alive is None:
            self._process_alive = self.manager.dict()
        if self._parent_map is None:
            self._parent_map = self.manager.dict()

        # 락/컨디션은 "진짜 multiprocessing 객체"로 (프록시 금지)
        if self._sync_lock is None:
            self._sync_lock = self._mp_ctx.RLock()
        if self._sync_cond is None:
            self._sync_cond = self._mp_ctx.Condition(self._sync_lock)

    def _collect_sync_flags(self, step: Step | None) -> set[str]:
        """Step 트리에서 모든 SyncStep의 flag 수집"""
        if step is None:
            return set()

        flags: set[str] = set()

        def walk(s: Step):
            # disabled step(및 그 하위)는 실행되지 않으므로 sync 참가자에서 제외한다.
            if getattr(s, "disabled", False):
                return

            if s.__class__.__name__ == "SyncStep":
                f = getattr(s, "flag", None)
                if f:
                    flags.add(str(f))

            for c in getattr(s, "children", []) or []:
                walk(c)

        walk(step)
        return flags

    def _register_process_sync_flags(
        self, process_id: str, step: Step | None, post_tree: Step | None = None
    ):
        """단일 프로세스의 sync flags를 등록/업데이트"""
        if self.manager is None:
            raise RuntimeError("Manager not initialized")

        flags = self._collect_sync_flags(step) | self._collect_sync_flags(post_tree)

        with self._sync_lock:
            # 각 flag에 이 프로세스 등록
            for flag in flags:
                # members 업데이트
                members = list(self._sync_members.get(flag, []))
                if process_id not in members:
                    members.append(process_id)
                self._sync_members[flag] = members

                # state 업데이트
                st = dict(self._sync_state.get(flag, {}))
                if not st:
                    st = {
                        "arrived": 0,
                        "phase": 0,
                        "parties_cur": len(members),
                        "parties_next": len(members),
                    }
                else:
                    # parties_next를 현재 멤버 수로 업데이트
                    st["parties_next"] = len(members)
                    # 새 멤버가 중간 합류하더라도 현재 라운드 참여자에 반영한다.
                    # 그렇지 않으면 기존 parties_cur 기준으로 barrier가 먼저 풀릴 수 있다.
                    st["parties_cur"] = len(members)

                self._sync_state[flag] = st

                print(
                    f"[SyncRegistry] Registered {process_id} to flag '{flag}', "
                    f"parties_next: {st['parties_next']}"
                )

        return flags

    def _unregister_process_sync_flags(self, process_id: str):
        if self.manager is None or self._sync_cond is None:
            return

        # Condition을 락으로 사용 (acquire/release 포함)
        with self._sync_cond:
            for flag in list(self._sync_members.keys()):
                members = list(self._sync_members.get(flag, []))
                if process_id not in members:
                    continue

                # members에서 제거
                members.remove(process_id)

                if not members:
                    # flag 자체 제거
                    del self._sync_members[flag]
                    if flag in self._sync_state:
                        del self._sync_state[flag]
                    continue

                # members 저장
                self._sync_members[flag] = members

                # sync_state 갱신
                st = dict(self._sync_state.get(flag, {}))
                parties_next = len(members)
                st["parties_next"] = parties_next

                # 현재 라운드 parties_cur도 현실에 맞춰 감소
                parties_cur = int(st.get("parties_cur", parties_next))
                if parties_cur > parties_next:
                    st["parties_cur"] = parties_next
                    parties_cur = parties_next

                arrived = int(st.get("arrived", 0))
                phase = int(st.get("phase", 0))

                # 제거 때문에 이미 조건이 충족 됐으면 라운드 종료 처리
                if parties_cur > 0 and arrived >= parties_cur:
                    st["phase"] = phase + 1
                    st["arrived"] = 0
                    # parties_cur는 다음 라운드 기준으로 스왑
                    st["parties_cur"] = parties_next

                self._sync_state[flag] = st

            # 대기중인 애들 깨우기
            self._sync_cond.notify_all()

    def _handle_main_tree_sync_unregister(self, pid: str):
        """메인 트리 sync flags 해제 처리 (CHANGE 타입 전환 시)"""
        if pid not in self.state_dicts:
            return

        # state_dict에 저장된 메인 트리의 sync flags 가져오기
        main_flags = set(self.state_dicts[pid].get("sync_flags", []))

        if not main_flags:
            print(f"[CHANGE] No main tree sync flags to unregister for {pid}", flush=True)
            return

        # 각 flag에서 해제
        if self._sync_cond is not None:
            with self._sync_cond:
                for flag in main_flags:
                    members = list(self._sync_members.get(flag, []))
                    if pid not in members:
                        continue

                    members.remove(pid)

                    if not members:
                        # 참가자가 없으면 flag 자체 삭제
                        del self._sync_members[flag]
                        if flag in self._sync_state:
                            del self._sync_state[flag]
                        print(f"[CHANGE] Removed empty flag '{flag}'", flush=True)
                        continue

                    self._sync_members[flag] = members

                    # state 업데이트
                    st = dict(self._sync_state.get(flag, {}))
                    parties_next = len(members)
                    st["parties_next"] = parties_next

                    parties_cur = int(st.get("parties_cur", parties_next))
                    if parties_cur > parties_next:
                        st["parties_cur"] = parties_next
                        parties_cur = parties_next

                    arrived = int(st.get("arrived", 0))
                    phase = int(st.get("phase", 0))

                    # 제거로 인해 조건 충족되면 라운드 종료
                    if parties_cur > 0 and arrived >= parties_cur:
                        st["phase"] = phase + 1
                        st["arrived"] = 0
                        st["parties_cur"] = parties_next

                    self._sync_state[flag] = st

                    print(
                        f"[CHANGE] Unregistered main tree flag '{flag}' for {pid}, "
                        f"remaining members: {len(members)}"
                    )

                self._sync_cond.notify_all()

        # state_dict의 sync_flags 초기화 (메인 트리 flags는 이제 없음)
        self.state_dicts[pid]["sync_flags"] = []

    def _handle_subtask_sync_register(self, pid: str, evt: dict):
        """서브태스크 sync flags 등록 처리"""
        if pid not in self.state_dicts:
            return

        # 이벤트에서 트리 정보 추출
        tree_dict = evt.get("sub_task_tree")
        post_tree_dict = evt.get("post_tree")
        subtask_type = evt.get("subtask_type", "INSERT")

        if not tree_dict:
            return

        # Step 객체 재구성
        sub_task_tree = Step.from_dict(tree_dict)
        post_tree = Step.from_dict(post_tree_dict) if post_tree_dict else None

        # 서브태스크의 sync flags 수집
        new_flags = self._collect_sync_flags(sub_task_tree) | self._collect_sync_flags(post_tree)

        if not new_flags:
            print(f"[SubTask-{subtask_type}] No sync flags found for {pid}")
            return

        # CHANGE 타입이면 같은 태스크를 실행 중인 다른 프로세스들도 찾아서 함께 등록
        if subtask_type == "CHANGE":
            # 모든 프로세스가 현재 어떤 태스크를 실행 중인지 확인
            task_id = sub_task_tree.step_id
            participating_pids = [pid]  # 일단 현재 프로세스는 포함

            # 다른 프로세스들 확인
            for other_pid, other_state in self.state_dicts.items():
                if other_pid == pid:
                    continue

                # 다른 프로세스의 현재 step 확인
                other_step = other_state.get("step", {})
                if other_step.get("stepId") == task_id:
                    # 같은 태스크를 실행 중
                    participating_pids.append(other_pid)
                    print(f"[CHANGE] Found {other_pid} also running task {task_id}", flush=True)

            # 모든 참여 프로세스를 flags에 등록
            with self._sync_lock:
                for flag in new_flags:
                    members = list(self._sync_members.get(flag, []))

                    # participating_pids의 모든 프로세스 추가
                    for p in participating_pids:
                        if p not in members:
                            members.append(p)

                    self._sync_members[flag] = members

                    # state 업데이트
                    st = dict(self._sync_state.get(flag, {}))
                    if not st:
                        st = {
                            "arrived": 0,
                            "phase": 0,
                            "parties_cur": len(members),
                            "parties_next": len(members),
                        }
                    else:
                        st["parties_next"] = len(members)
                        st["parties_cur"] = len(members)

                    self._sync_state[flag] = st

                    print(
                        f"[CHANGE] Registered flag '{flag}' with {len(members)} participants: {members}",
                    )

            # CHANGE 타입이면 state_dict의 sync_flags 업데이트
            self.state_dicts[pid]["sync_flags"] = list(new_flags)

        else:
            # INSERT 타입은 기존 로직 사용
            sync_flags = self._register_process_sync_flags(pid, sub_task_tree, post_tree)
            print(
                f"[SubTask-{subtask_type}] Registered sync flags for {pid}: {sync_flags}"
            )

    def _handle_subtask_sync_unregister(self, pid: str, evt: dict):
        """서브태스크 sync flags 해제 처리"""
        if pid not in self.state_dicts:
            return

        # 이벤트에서 트리 정보 추출
        tree_dict = evt.get("sub_task_tree")
        post_tree_dict = evt.get("post_tree")

        if not tree_dict:
            return

        # Step 객체 재구성
        sub_task_tree = Step.from_dict(tree_dict)
        post_tree = Step.from_dict(post_tree_dict) if post_tree_dict else None

        # 서브태스크의 flags 수집 (tree + post_tree 모두 포함)
        flags = self._collect_sync_flags(sub_task_tree) | self._collect_sync_flags(post_tree)

        if not flags:
            return

        # 각 flag에서 해제
        if self._sync_cond is not None:
            with self._sync_cond:
                for flag in flags:
                    members = list(self._sync_members.get(flag, []))
                    if pid not in members:
                        continue

                    members.remove(pid)

                    if not members:
                        del self._sync_members[flag]
                        if flag in self._sync_state:
                            del self._sync_state[flag]
                        continue

                    self._sync_members[flag] = members

                    st = dict(self._sync_state.get(flag, {}))
                    parties_next = len(members)
                    st["parties_next"] = parties_next

                    parties_cur = int(st.get("parties_cur", parties_next))
                    if parties_cur > parties_next:
                        st["parties_cur"] = parties_next
                        parties_cur = parties_next

                    arrived = int(st.get("arrived", 0))
                    phase = int(st.get("phase", 0))

                    if parties_cur > 0 and arrived >= parties_cur:
                        st["phase"] = phase + 1
                        st["arrived"] = 0
                        st["parties_cur"] = parties_next

                    self._sync_state[flag] = st

                self._sync_cond.notify_all()

        print(
            f"[SubTask-{evt.get('subtask_type')}] Unregistered sync flags for {pid}: {flags}",
        )

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

        # sync_state 업데이트
        with self._sync_lock:
            # 이번 배치에서 등장한 모든 flag 순회
            all_flags = {flag for flags in pid_flags.values() for flag in flags}
            for flag in all_flags:
                # members - 일반 list로 저장 (proxy 중첩 금지)
                members = list(self._sync_members.get(flag, []))

                # 이번 배치의 pid들만 추가
                for pid, flags in pid_flags.items():
                    if flag in flags and pid not in members:
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

    def _spawn_event_sub_task(self, parent_pid: str, evt: dict):
        if parent_pid not in self.state_dicts:
            return

        event_tree_dict = evt.get("event_tree")
        event_task_id = evt.get("event_task_id")
        step_id = evt.get("step_id")
        run_mode = str(evt.get("run_mode") or "ASYNC").upper()
        call_seq = evt.get("call_seq")

        if not event_task_id:
            raise ValueError("event_sub_task_start event requires event_task_id")
        if not isinstance(event_tree_dict, dict):
            raise ValueError("event_sub_task_start event requires event_tree")
        if run_mode not in ("SYNC", "ASYNC"):
            raise ValueError("event_sub_task_start event requires valid run_mode")
        parent_state = self.state_dicts[parent_pid]

        def reject_spawn(message: str):
            pending_map = dict(parent_state.get("event_start_pending_map", {}))
            if (
                event_task_id in pending_map
                and call_seq is not None
                and pending_map[event_task_id] == call_seq
            ):
                pending_map.pop(event_task_id, None)
                parent_state["event_start_pending_map"] = pending_map

            rejected_map = dict(parent_state.get("event_start_rejected_map", {}))
            rejected_map[event_task_id] = {
                "call_seq": call_seq,
                "message": message,
            }
            parent_state["event_start_rejected_map"] = rejected_map

            q = self.result_queues.get(parent_pid)
            if q is not None:
                q.put(
                    {
                        "type": "error",
                        "process_id": parent_pid,
                        "step_id": step_id or parent_state.get("current_step_id"),
                        "ts": time.time(),
                        "generation": self._pid_generation.get(parent_pid, parent_state.get("generation")),
                        "error": message,
                    }
                )

        # 이미 동일 이벤트 프로세스가 살아있으면(실행/일시정지 포함) 즉시 에러 처리
        existing_proc = self.processes.get(event_task_id)
        if existing_proc is not None and existing_proc.is_alive():
            reject_spawn(f"Event sub task already running: {event_task_id}")
            return

        event_tree = Step.from_dict(event_tree_dict)

        args = MakeProcessArgs(
            process_id=event_task_id,
            step=event_tree,
            repeat_count=1,
            robot_model=parent_state.get("robot_model"),
            category=parent_state.get("category"),
            step_mode=False,
            min_step_interval=None,
            is_ui_execution=parent_state.get("is_ui_execution", False),
            post_tree=None,
            parent_process_id=parent_pid,
        )

        p = self.make_process(args, multi_start=False, start_event=None, invoke_callbacks=True)
        if p is None:
            reject_spawn(f"Failed to spawn event sub task: {event_task_id}")
            return

        # post_tree 대기 판단에 사용할 이벤트 자식 메타데이터 기록
        if event_task_id in self.state_dicts:
            self.state_dicts[event_task_id]["is_event_sub_task"] = True
            self.state_dicts[event_task_id]["event_run_mode"] = run_mode

        # start 이전에 등록해야 자식이 즉시 종료해도 resume 누락이 없음
        if run_mode == "SYNC":
            parent_children = list(self.state_dicts[parent_pid].get("event_sync_children", []))
            if event_task_id not in parent_children:
                parent_children.append(event_task_id)
                self.state_dicts[parent_pid]["event_sync_children"] = parent_children

            self._event_wait_parent_by_child[event_task_id] = parent_pid
            self._event_wait_children_count[parent_pid] = (
                self._event_wait_children_count.get(parent_pid, 0) + 1
            )

        try:
            p.start()
        except Exception as e:
            # start 실패 시 SYNC 대기 카운트/매핑 롤백 (parent deadlock 방지)
            if run_mode == "SYNC":
                self._event_wait_parent_by_child.pop(event_task_id, None)
                remaining = self._event_wait_children_count.get(parent_pid, 0) - 1
                if remaining <= 0:
                    self._event_wait_children_count.pop(parent_pid, None)
                else:
                    self._event_wait_children_count[parent_pid] = remaining

                if parent_pid in self.state_dicts:
                    parent_children = list(self.state_dicts[parent_pid].get("event_sync_children", []))
                    if event_task_id in parent_children:
                        parent_children.remove(event_task_id)
                        self.state_dicts[parent_pid]["event_sync_children"] = parent_children

            if self._process_alive is not None:
                self._process_alive[event_task_id] = False
            if self._parent_map is not None and event_task_id in self._parent_map:
                del self._parent_map[event_task_id]

            # 생성 중 만들었던 리소스 정리
            self.completion_events.pop(event_task_id, None)
            q = self.result_queues.pop(event_task_id, None)
            if q is not None:
                with contextlib.suppress(Exception):
                    q.close()
            self.pause_events.pop(event_task_id, None)
            self.resume_events.pop(event_task_id, None)
            self.stop_events.pop(event_task_id, None)
            self.start_events.pop(event_task_id, None)
            self._pid_generation.pop(event_task_id, None)

            reject_spawn(f"Failed to start event sub task {event_task_id}: {e!s}")
            return

        self.processes[event_task_id] = p
        if call_seq is not None:
            self._event_start_seq_by_child[event_task_id] = int(call_seq)

    def make_process(
        self,
        args: MakeProcessArgs,
        *,
        multi_start: bool = False,
        start_event: EventType | None = None,
        invoke_callbacks: bool = True,
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
        parent_process_id = args.get("parent_process_id")
        event_sub_tree_list = args.get("event_sub_tree_list") or []

        # 이미 실행 중인 프로세스 체크
        if process_id in self.processes and self.processes[process_id].is_alive():
            # state_dict 존재 확인
            if process_id not in self.state_dicts:
                return None

            # 실행 중인 프로세스가 step_mode면 토글
            is_step_mode = self.state_dicts[process_id].get("step_mode", False)

            if is_step_mode:
                current_state = self.state_dicts[process_id].get("state")

                if current_state == RB_Flow_Manager_ProgramState.PAUSED:
                    self.resume(process_id)
                elif current_state == RB_Flow_Manager_ProgramState.RUNNING:
                    self.pause(process_id)
                elif current_state == RB_Flow_Manager_ProgramState.WAITING:
                    self.resume(process_id)

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

        # 이벤트 백프레셔로 worker가 put()에서 멈추지 않도록 bounded queue를 피한다.
        self.result_queues[process_id] = self._mp_ctx.Queue()
        if self._process_alive is not None:
            self._process_alive[process_id] = True
        if self._parent_map is not None:
            self._parent_map[process_id] = parent_process_id

        # 단일 프로세스의 경우 sync flags 등록 (배치가 아닐 때)
        if not multi_start:
            sync_flags = self._register_process_sync_flags(process_id, step, post_tree)
            self.state_dicts[process_id]["sync_flags"] = list(sync_flags)

        # 상태 정보 업데이트
        self.state_dicts[process_id]["state"] = RB_Flow_Manager_ProgramState.RUNNING
        self.state_dicts[process_id]["current_step_id"] = None
        self.state_dicts[process_id]["robot_model"] = robot_model
        self.state_dicts[process_id]["category"] = category
        self.state_dicts[process_id]["generation"] = gen
        self.state_dicts[process_id]["step_mode"] = step_mode
        self.state_dicts[process_id]["is_ui_execution"] = is_ui_execution
        self.state_dicts[process_id]["parent_process_id"] = parent_process_id
        self.state_dicts[process_id]["executor_managed"] = True
        self.state_dicts[process_id]["event_sub_tree_list"] = [
            t.to_dict() if hasattr(t, "to_dict") else t for t in event_sub_tree_list
        ]

        # 콜백 호출
        if invoke_callbacks:
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
                "process_alive": self._process_alive,
                "parent_map": self._parent_map,
                "all_state_dicts": self.state_dicts,
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
                invoke_callbacks=False,
            )
            if p:
                processes.append(p)
                pids.append(pid)

        # 배치 실행에서는 init을 한 번만 호출해 중복 초기화 비용을 줄인다.
        if self._on_init is not None:
            self._on_init(self.state_dicts)
        if self.controller is not None:
            self.controller.on_init(self.state_dicts)

        # start 콜백은 프로세스 단위로 호출한다.
        for pid in pids:
            if self._on_start is not None:
                self._on_start(pid)
            if self.controller is not None:
                self.controller.on_start(pid)

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
                print(f"Ignoring stale event: {evt}")
                continue

            if self.state_dicts.get(pid) is None:
                print(f"Process not found: {pid}")
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
        if evt_type == "next":
            if step_id is None:
                raise ValueError("next event requires step_id")
            if self._on_next is not None:
                self._on_next(pid, step_id)
            if self.controller is not None:
                self.controller.on_next(pid, step_id)

        elif evt_type == "sub_task_start":
            if sub_task_id is None or sub_task_type not in ("INSERT", "CHANGE"):
                raise ValueError("sub_task_start event requires sub_task_id and valid sub_task_type")
            if self._on_sub_task_start is not None:
                self._on_sub_task_start(pid, sub_task_id, sub_task_type)
            if self.controller is not None:
                self.controller.on_sub_task_start(pid, sub_task_id, sub_task_type)

        elif evt_type == "sub_task_done":
            if sub_task_id is None or sub_task_type not in ("INSERT", "CHANGE"):
                raise ValueError("sub_task_done event requires sub_task_id and valid sub_task_type")
            if self._on_sub_task_done is not None:
                self._on_sub_task_done(pid, sub_task_id, sub_task_type)
            if self.controller is not None:
                self.controller.on_sub_task_done(pid, sub_task_id, sub_task_type)

        elif evt_type == "subtask_sync_register":
            self._handle_subtask_sync_register(pid, evt)

        elif evt_type == "subtask_sync_unregister":
            self._handle_subtask_sync_unregister(pid, evt)

        elif evt_type == "main_tree_sync_unregister":
            self._handle_main_tree_sync_unregister(pid)

        elif evt_type == "event_sub_task_start":
            self._spawn_event_sub_task(pid, evt)

        elif evt_type == "done":
            if step_id is None:
                raise ValueError("done event requires step_id")
            if self._on_done is not None:
                self._on_done(pid, step_id)
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
            if step_id is None:
                raise ValueError("pause event requires step_id")
            state = (
                RB_Flow_Manager_ProgramState.WAITING
                if is_wait
                else RB_Flow_Manager_ProgramState.PAUSED
            )
            self.state_dicts[pid]["state"] = state

            # event sub task(SYNC) 자식이 이미 끝난 뒤 WAITING으로 진입한 경우,
            # 누락된 resume를 즉시 보정한다.
            if is_wait and self._event_wait_resume_pending.get(pid, 0) > 0:
                self._event_wait_resume_pending[pid] -= 1
                if self._event_wait_resume_pending[pid] <= 0:
                    self._event_wait_resume_pending.pop(pid, None)
                self.resume(pid)
                return

            # step_mode barrier 대기(WAITING)는 해당 프로세스만 대기시킨다.
            # 부모/자식 전파는 사용자 pause 요청(PAUSED)일 때만 수행.
            if not is_wait:
                parent_process_id = evt.get("parent_process_id")

                if parent_process_id is not None:
                    self.pause(parent_process_id)

                for find_pid in list(self.processes.keys()):
                    if self.state_dicts.get(find_pid, {}).get("parent_process_id") == pid:
                        self.pause(find_pid)

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
            if step_id is None:
                raise ValueError("resume event requires step_id")
            # post 단계는 상태를 POST_START로 유지한다(표시/흐름 꼬임 방지).
            if self.state_dicts[pid].get("state") != RB_Flow_Manager_ProgramState.POST_START:
                self.state_dicts[pid]["state"] = RB_Flow_Manager_ProgramState.RUNNING

            parent_process_id = evt.get("parent_process_id")
            if parent_process_id is not None:
                # 자식의 resume를 부모로 전파할 때는,
                # 부모가 실제 대기 상태(PAUSED/WAITING)인 경우에만 깨운다.
                # POST_START 상태를 RUNNING으로 덮어써 current_step_id가 과거 step으로
                # 되돌아가는 현상을 방지한다.
                parent_state = self.state_dicts.get(parent_process_id, {}).get("state")
                if parent_state in (
                    RB_Flow_Manager_ProgramState.PAUSED,
                    RB_Flow_Manager_ProgramState.WAITING,
                ):
                    self.resume(parent_process_id)

            for find_pid in list(self.processes.keys()):
                if self.state_dicts.get(find_pid, {}).get("parent_process_id") == pid:
                    self.resume(find_pid)

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

        elif evt_type == "exec_ready":
            parent_pid = self.state_dicts.get(pid, {}).get("parent_process_id")
            if parent_pid is not None and parent_pid in self.state_dicts:
                pending_map = dict(self.state_dicts[parent_pid].get("event_start_pending_map", {}))
                seq = self._event_start_seq_by_child.get(pid)
                if pid in pending_map and (seq is None or pending_map[pid] == seq):
                    pending_map.pop(pid, None)
                    self.state_dicts[parent_pid]["event_start_pending_map"] = pending_map

    def _start_monitor(self):
        """백그라운드 모니터 스레드 시작"""
        self._monitor_thread = Thread(target=self._monitor_processes, daemon=True)
        self._monitor_thread.start()

    def _has_alive_descendant_process(self, parent_pid: str) -> bool:
        # parent_process_id 그래프를 따라 자식/자손 중 alive가 있으면 True
        stack = [parent_pid]
        visited: set[str] = set()

        while stack:
            cur_parent = stack.pop()
            if cur_parent in visited:
                continue
            visited.add(cur_parent)

            for pid, state in self.state_dicts.items():
                if pid == cur_parent:
                    continue
                if state.get("parent_process_id") != cur_parent:
                    continue

                # worker finally에서 _process_alive를 false로 내리므로 종료 판정을 더 정확히 본다.
                if self._process_alive is not None and self._process_alive.get(pid, False):
                    return True

                proc = self.processes.get(pid)
                if proc is not None and proc.is_alive():
                    return True

                stack.append(pid)

        return False

    def _flush_complete_callbacks(self):
        if not self._pending_complete_callbacks:
            return

        for pid in list(self._pending_complete_callbacks):
            if self._has_alive_descendant_process(pid):
                continue
            if self._on_complete is not None:
                self._on_complete(pid)
            if self.controller is not None:
                self.controller.on_complete(pid)
            self._pending_complete_callbacks.discard(pid)

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
                self._pending_complete_callbacks.add(pid)
                self._cleanup_process(pid)

            # 부모-자식 관계를 고려해 on_complete 지연 호출
            self._flush_complete_callbacks()

            # 모든 프로세스 완료 시
            if not self.processes:
                # 남은 pending callback 강제 flush
                self._flush_complete_callbacks()
                if self._on_all_complete is not None:
                    self._on_all_complete()
                if self.controller is not None:
                    self.controller.on_all_complete()

                self._auto_cleanup()
                break

            time.sleep(POLLING_INTERVAL)

    def _cleanup_process(self, process_id: str):
        p = self.processes.get(process_id)

        # 프로세스가 없거나 이미 죽었으면 무조건 unregister
        dead = (p is None) or (not p.is_alive())

        if dead:
            self._unregister_process_sync_flags(process_id)
            if self._process_alive is not None:
                self._process_alive[process_id] = False
            # 비정상 종료/이벤트 누락 시 state가 RUNNING/WAITING 등에 머물 수 있다.
            # 이 경우 부모 post_tree 대기 루프가 영구 대기하지 않도록 terminal state로 정규화한다.
            sd = self.state_dicts.get(process_id)
            if sd is not None:
                cur_state = sd.get("state")
                if cur_state not in (
                    RB_Flow_Manager_ProgramState.COMPLETED,
                    RB_Flow_Manager_ProgramState.STOPPED,
                    RB_Flow_Manager_ProgramState.ERROR,
                ):
                    sd["state"] = RB_Flow_Manager_ProgramState.STOPPED

        with self._mu:
            p = self.processes.pop(process_id, None)
            if p:
                p.join(timeout=0.1)

            self.completion_events.pop(process_id, None)

            q = self.result_queues.pop(process_id, None)
            if q:
                with contextlib.suppress(Exception):
                    q.close()

            self.pause_events.pop(process_id, None)
            self.resume_events.pop(process_id, None)
            self.stop_events.pop(process_id, None)
            self.start_events.pop(process_id, None)

            self._pid_generation.pop(process_id, None)

        wait_parent = self._event_wait_parent_by_child.pop(process_id, None)
        start_seq = self._event_start_seq_by_child.pop(process_id, None)
        if wait_parent is not None:
            if wait_parent in self.state_dicts:
                parent_children = list(self.state_dicts[wait_parent].get("event_sync_children", []))
                if process_id in parent_children:
                    parent_children.remove(process_id)
                    self.state_dicts[wait_parent]["event_sync_children"] = parent_children

            remaining = self._event_wait_children_count.get(wait_parent, 0) - 1
            if remaining <= 0:
                self._event_wait_children_count.pop(wait_parent, None)

                # 상태 체크를 제거하고 무조건 resume 시도
                if wait_parent in self.processes:
                    self.resume(wait_parent)  # ← 이렇게 수정
                else:
                    self._event_wait_resume_pending[wait_parent] = (
                        self._event_wait_resume_pending.get(wait_parent, 0) + 1
                    )

        parent_pid = self.state_dicts.get(process_id, {}).get("parent_process_id")
        if parent_pid is not None and parent_pid in self.state_dicts:
            pending_map = dict(self.state_dicts[parent_pid].get("event_start_pending_map", {}))
            if (
                process_id in pending_map
                and (start_seq is None or pending_map[process_id] == start_seq)
            ):
                pending_map.pop(process_id, None)
                self.state_dicts[parent_pid]["event_start_pending_map"] = pending_map

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
        self._pending_complete_callbacks.clear()
        self._event_wait_parent_by_child.clear()
        self._event_wait_children_count.clear()
        self._event_wait_resume_pending.clear()
        self._event_start_seq_by_child.clear()

        if self._sync_state is not None:
            self._sync_state.clear()
        if self._sync_members is not None:
            self._sync_members.clear()
        if self._process_alive is not None:
            self._process_alive.clear()
        if self._parent_map is not None:
            self._parent_map.clear()

        # 락/컨디션은 stuck 가능성 때문에 재생성
        self._sync_lock = self._mp_ctx.RLock()
        self._sync_cond = self._mp_ctx.Condition(self._sync_lock)

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

        # clear 후 set (pulse 패턴)
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
            print("All processes paused")

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
        if self.state_dicts[process_id].get("state") != RB_Flow_Manager_ProgramState.POST_START:
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

        # 모든 프로세스가 중지 상태일 때만 all_stop 콜백 호출
        if all(
            self.state_dicts[pid].get("state") == RB_Flow_Manager_ProgramState.STOPPED
            for pid in self.processes
        ):
            if self._on_all_stop is not None:
                self._on_all_stop()
            if self.controller is not None:
                self.controller.on_all_stop()

        print("Process stopped", flush=True)

        return True

    def stop_all(self):
        """모든 스크립트 중지"""
        pids = list(self.processes.keys())
        for process_id in pids:
            self.stop(process_id)

        # soft stop 이후에도 남아있는 프로세스는 강제 종료
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if all(not p.is_alive() for p in self.processes.values()):
                break
            time.sleep(0.05)

        for process_id in pids:
            p = self.processes.get(process_id)
            if p is not None and p.is_alive():
                with contextlib.suppress(Exception):
                    p.terminate()

    def pause_all(self):
        """모든 스크립트 일시정지"""
        for process_id in list(self.processes.keys()):
            self.pause(process_id)

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
