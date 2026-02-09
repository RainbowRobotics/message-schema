import time
from collections.abc import MutableMapping
from multiprocessing.managers import DictProxy, ListProxy
from typing import Any, Literal

from rb_flat_buffers.flow_manager.RB_Flow_Manager_ProgramState import (
    RB_Flow_Manager_ProgramState,
)
from rb_flat_buffers.flow_manager.Request_Update_All_Step_State import (
    Request_Update_All_Step_StateT,
)
from rb_flat_buffers.flow_manager.Request_Update_Executor_State import (
    Request_Update_Executor_StateT,
)
from rb_flat_buffers.flow_manager.Request_Update_Step_State import Request_Update_Step_StateT
from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_MotionResume import Request_MotionResumeT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.program.RB_Program_Sub_Task_Type import RB_Program_Sub_Task_Type
from rb_flat_buffers.program.Request_Program_At_End import Request_Program_At_EndT
from rb_flat_buffers.program.Request_Program_At_Start import Request_Program_At_StartT
from rb_flat_buffers.program.Request_Update_Sub_Task_State import Request_Update_Sub_Task_StateT
from rb_flow_manager.controller.base_controller import BaseController
from rb_modules.log import rb_log
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError


class Zenoh_Controller(BaseController):
    def __init__(self):
        self._zenoh_client: ZenohClient | None = None
        self._state_dicts: dict[str, MutableMapping[str, Any]] = {}

    def on_init(self, state_dicts):
        self._state_dicts = state_dicts

        # 이미 생성되어 있으면 재사용
        if self._zenoh_client is None:
            self._zenoh_client = ZenohClient()

            if self._zenoh_client is None:
                raise RuntimeError("Zenoh client not initialized")

            # 세션 대기를 비동기적으로 처리 (블로킹 최소화)
            # 세션이 없어도 publish는 내부 큐에 쌓여서 나중에 전송됨
            start_time = time.time()

            while self._zenoh_client.session is None:
                # 0.5초만 기다리고 진행 (3초 → 0.5초)
                if time.time() - start_time > 0.5:
                    print("[WARNING] Zenoh session not ready, continuing anyway", flush=True)
                    break
                time.sleep(0.01)  # 100ms → 10ms로 단축

        # 초기 상태 업데이트 시도 (실패해도 계속 진행)
        for task_id in self._state_dicts:
            try:
                self.update_step_state("", task_id, RB_Flow_Manager_ProgramState.RUNNING)
            except (ZenohNoReply, ZenohReplyError) as e:
                rb_log.warning(f"initial state update skipped: {e}")
            except RuntimeError as e:
                rb_log.warning(f"initial state update skipped: {e}")
            except ValueError as e:
                rb_log.warning(f"initial state update skipped: {e}")
            except TypeError as e:
                # 초기화 단계에서는 에러 무시
                rb_log.warning(f"initial state update skipped: {e}")

    def on_start(self, task_id: str) -> None:
        if self._zenoh_client is not None:
            req = Request_Program_At_StartT()
            req.taskId = task_id
            self._zenoh_client.publish("rrs/program/at_start", flatbuffer_req_obj=req, flatbuffer_buf_size=8)

        self.update_step_state("", task_id, RB_Flow_Manager_ProgramState.RUNNING)
        self.update_executor_state(RB_Flow_Manager_ProgramState.RUNNING)

    def on_stop(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.STOPPED)

        robot_model = self._state_dicts.get(task_id, {}).get("robot_model", "*")

        try:
            if self._zenoh_client is not None:
                self._zenoh_client.query_one(
                    f"{robot_model}/call_halt",
                    flatbuffer_req_obj=Request_MotionHaltT(),
                    flatbuffer_res_T_class=Response_FunctionsT,
                    flatbuffer_buf_size=2,
                )
        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.warning(f"Warning program: {e}")

    def on_wait(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.WAITING)
        self.update_executor_state(RB_Flow_Manager_ProgramState.WAITING)

        try:
            robot_model = self._state_dicts.get(task_id, {}).get("robot_model", "*")

            if self._zenoh_client is not None:
                self._zenoh_client.query_one(
                    f"{robot_model}/call_pause",
                    flatbuffer_req_obj=Request_MotionPauseT(),
                    flatbuffer_res_T_class=Response_FunctionsT,
                    flatbuffer_buf_size=2,
                )

        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.warning(f"Warning program: {e}")

    def on_pause(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.PAUSED)
        self.update_executor_state(RB_Flow_Manager_ProgramState.PAUSED)

        try:
            robot_model = self._state_dicts.get(task_id, {}).get("robot_model", "*")

            if self._zenoh_client is not None:
                self._zenoh_client.query_one(
                    f"{robot_model}/call_pause",
                    flatbuffer_req_obj=Request_MotionPauseT(),
                    flatbuffer_res_T_class=Response_FunctionsT,
                    flatbuffer_buf_size=2,
                )
        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.warning(f"Warning program: {e}")

    def on_resume(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.RUNNING)
        self.update_executor_state(RB_Flow_Manager_ProgramState.RUNNING)

        try:
            robot_model = self._state_dicts.get(task_id, {}).get("robot_model", "*")

            if self._zenoh_client is not None:
                self._zenoh_client.query_one(
                    f"{robot_model}/call_resume",
                    flatbuffer_req_obj=Request_MotionResumeT(),
                    flatbuffer_res_T_class=Response_FunctionsT,
                    flatbuffer_buf_size=2,
                )

        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.warning(f"Warning program: {e}")

    def on_next(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.RUNNING)

    def on_sub_task_start(self, task_id: str, sub_task_id: str, sub_task_type: RB_Program_Sub_Task_Type) -> None:
        self.sub_task_start_or_done(task_id, sub_task_id, sub_task_type, RB_Flow_Manager_ProgramState.RUNNING)

    def on_sub_task_done(self, task_id: str, sub_task_id: str, sub_task_type: RB_Program_Sub_Task_Type) -> None:
        self.sub_task_start_or_done(task_id, sub_task_id, sub_task_type, RB_Flow_Manager_ProgramState.COMPLETED)

    def on_error(self, task_id: str, step_id: str, error: Exception) -> None:
        str_error = str(error)
        self.update_step_state(
            step_id, task_id, RB_Flow_Manager_ProgramState.ERROR, error=str_error
        )
        self.update_executor_state(RB_Flow_Manager_ProgramState.ERROR, error=str_error)

    def on_done(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.COMPLETED)

    def on_post_start(self, task_id: str) -> None:
        self.update_executor_state(state=RB_Flow_Manager_ProgramState.AFTER_COMPLETED)

    def on_complete(self, task_id: str) -> None:
        self.update_step_state("", task_id, RB_Flow_Manager_ProgramState.IDLE)
        self.update_all_task_step_state(task_id, RB_Flow_Manager_ProgramState.IDLE)

        if self._zenoh_client is not None:
            req = Request_Program_At_EndT()
            req.taskId = task_id
            self._zenoh_client.publish("rrs/program/at_end", flatbuffer_req_obj=req, flatbuffer_buf_size=8)

    def on_close(self) -> None:
        pass

    def on_all_complete(self) -> None:
        self.update_executor_state(state=RB_Flow_Manager_ProgramState.IDLE)

    def on_all_stop(self) -> None:
        self.update_executor_state(state=RB_Flow_Manager_ProgramState.STOPPED)

    def on_all_pause(self) -> None:
        self.update_executor_state(state=RB_Flow_Manager_ProgramState.PAUSED)

    def update_step_state(
        self, step_id: str, task_id: str, state: int, error: str | None = ""
    ) -> None:
        """Step 상태 업데이트"""
        try:
            if self._zenoh_client is not None:
                req = Request_Update_Step_StateT()
                req.stepId = step_id
                req.taskId = task_id
                req.state = state
                req.error = error

                self._zenoh_client.publish(
                    "rrs/step/update_state", flatbuffer_req_obj=req, flatbuffer_buf_size=256
                )
        except Exception as e:
            rb_log.error(f"Error updating step state: {e}")
            raise

    def update_all_task_step_state(self, task_id: str, state: int) -> None:
        """Task의 모든 Step 상태 업데이트"""
        try:
            if self._zenoh_client is not None:
                req = Request_Update_All_Step_StateT()
                req.taskId = task_id
                req.state = state

                self._zenoh_client.publish(
                    "rrs/task/update_all_step_state",
                    flatbuffer_req_obj=req,
                    flatbuffer_buf_size=32,
                )
        except Exception as e:
            rb_log.error(f"Error updating all task step state: {e}")
            raise

    def update_executor_state(self, state: int, error: str | None = "") -> None:
        """Executor 상태 업데이트"""
        try:
            if self._zenoh_client is not None:
                req = Request_Update_Executor_StateT()
                req.state = state
                req.error = error
                self._zenoh_client.publish(
                    "rrs/executor/state",
                    flatbuffer_req_obj=req,
                    flatbuffer_buf_size=256 if error is not None else 32,
                )
        except Exception as e:
            rb_log.error(f"Error updating executor state: {e}")
            raise

    def sub_task_start_or_done(self, task_id: str, sub_task_id: str, sub_task_type: Literal["INSERT", "CHANGE"], state: int) -> None:
        try:
            sub_task_type_enum = RB_Program_Sub_Task_Type.SUB_TASK_INSERT if sub_task_type == "INSERT" else RB_Program_Sub_Task_Type.SUB_TASK_CHANGE

            if self._zenoh_client is not None:
                req = Request_Update_Sub_Task_StateT()
                req.subTaskId = sub_task_id
                req.subTaskType = sub_task_type_enum
                req.state = state
                self._zenoh_client.publish(
                    f"rrs/program/{task_id}/sub_task/update_state",
                    flatbuffer_req_obj=req,
                    flatbuffer_buf_size=32,
                )
        except Exception as e:
            rb_log.error(f"Error updating sub task state: {e}")
            raise

    def _proxy_to_dict(self, obj):
        if isinstance(obj, dict):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, DictProxy):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, ListProxy):
            return [self._proxy_to_dict(v) for v in obj]

        return obj
