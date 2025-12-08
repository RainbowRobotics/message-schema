import time
from collections.abc import MutableMapping
from multiprocessing.managers import DictProxy, ListProxy
from typing import Any

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
from rb_flow_manager.controller.base_controller import BaseController
from rb_zenoh.client import ZenohClient


class Zenoh_Controller(BaseController):
    def __init__(self):
        self._zenoh_client: ZenohClient | None = None
        self._state_dicts: dict[str, MutableMapping[str, Any]] = {}

    def on_init(self, state_dicts):
        self._state_dicts = state_dicts
        self._zenoh_client = ZenohClient()

        if self._zenoh_client is None:
            raise RuntimeError("Zenoh client not initialized")

        start_time = time.time()

        while self._zenoh_client.session is None:
            if time.time() - start_time > 3.0:
                raise RuntimeError("Zenoh session not established within 3 seconds.")
            time.sleep(0.1)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/stop", payload={})

    def on_start(self, task_id: str) -> None:
        self.update_executor_state(RB_Flow_Manager_ProgramState.RUNNING)

    def on_stop(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.STOPPED)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/stop", payload={})

    def on_wait(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.WAITING)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/pause", payload={})

    def on_pause(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.PAUSED)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/pause", payload={})

    def on_resume(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.RUNNING)
        self.update_executor_state(RB_Flow_Manager_ProgramState.RUNNING)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/resume", payload={})

    def on_next(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.RUNNING)

    def on_error(self, task_id: str, step_id: str, error: Exception) -> None:
        str_error = str(error)
        self.update_step_state(
            step_id, task_id, RB_Flow_Manager_ProgramState.ERROR, error=str_error
        )
        self.update_executor_state(RB_Flow_Manager_ProgramState.ERROR, error=str_error)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/stop", payload={})

    def on_done(self, task_id: str, step_id: str) -> None:
        self.update_step_state(step_id, task_id, RB_Flow_Manager_ProgramState.COMPLETED)

    def on_complete(self, task_id: str) -> None:
        self.update_all_task_step_state(task_id, RB_Flow_Manager_ProgramState.IDLE)

    def on_close(self) -> None:
        # if self._zenoh_client is not None and not self._zenoh_client.is_current_process_client():
        #     self._zenoh_client.close()
        #     self._zenoh_client = None

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/stop", payload={})

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
            print(f"Error updating step state: {e}")
            raise e

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
            print(f"Error updating all task step state: {e}")
            raise e

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
            print(f"Error updating executor state: {e}")
            raise e

    def _proxy_to_dict(self, obj):
        if isinstance(obj, dict):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, DictProxy):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, ListProxy):
            return [self._proxy_to_dict(v) for v in obj]

        return obj
