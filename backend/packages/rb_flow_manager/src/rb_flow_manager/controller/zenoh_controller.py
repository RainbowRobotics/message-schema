import asyncio
import time
from collections.abc import (
    Mapping,
)
from multiprocessing.managers import (
    DictProxy,
    ListProxy,
)
from typing import Any

from rb_flat_buffers.flow_manager.RB_Flow_Manager_ProgramState import (
    RB_Flow_Manager_ProgramState,
)
from rb_flat_buffers.flow_manager.Request_Update_All_Step_State import (
    Request_Update_All_Step_StateT,
)
from rb_flat_buffers.flow_manager.Request_Update_Step_State import (
    Request_Update_Step_StateT,
)
from rb_flow_manager.controller.base_controller import (
    BaseController,
)
from rb_modules.log import (
    rb_log,
)
from rb_zenoh.client import (
    ZenohClient,
)


class Zenoh_Controller(BaseController):
    def __init__(self):
        self._zenoh_client: ZenohClient | None = None
        self._state_dicts: dict[str, Mapping[str, Any]] = {}
        self._bg_task: asyncio.Task | None = None

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

        # loop = asyncio.get_event_loop()
        # self._bg_task = loop.create_task(self._publish_executor_state())

    def on_start(self, process_id: str) -> None:
        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/resume", payload={})

    def on_stop(self, process_id: str, step_id: str) -> None:
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.STOPPED)

        # TODO: zenoh publish stop

    def on_pause(self, process_id: str, step_id: str) -> None:
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.PAUSED)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/pause", payload={})

    def on_resume(self, process_id: str, step_id: str) -> None:
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.RUNNING)

        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/resume", payload={})

    def on_next(self, process_id: str, step_id: str) -> None:
        print("on_next >>>", process_id, step_id, flush=True)
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.RUNNING)

    def on_error(self, process_id: str, step_id: str, error: Exception) -> None:
        rb_log.error(f"on_error >>> {process_id} {step_id} {error}")
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.ERROR)

    def on_complete(self, process_id: str, step_id: str) -> None:
        rb_log.info(f"on_complete >>> {process_id} {step_id}")
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.COMPLETED)

    def on_process_complete(self, process_id: str) -> None:
        self.update_all_task_step_state(process_id, RB_Flow_Manager_ProgramState.IDLE)

    def on_close(self) -> None:
        if self._zenoh_client is not None and not self._zenoh_client.is_current_process_client():
            self._zenoh_client.close()
            self._zenoh_client = None

        if self._bg_task is not None:
            self._bg_task.cancel()
            self._bg_task = None

    def on_all_complete(self) -> None:
        pass

    def on_all_stop(self) -> None:
        pass

    def on_all_pause(self) -> None:
        pass

    def update_step_state(self, step_id: str, state: RB_Flow_Manager_ProgramState) -> None:
        try:
            if self._zenoh_client is not None:
                req = Request_Update_Step_StateT()
                req.stepId = step_id
                req.state = state

                self._zenoh_client.publish(
                    "rrs/step/update_state", flatbuffer_req_obj=req, flatbuffer_buf_size=256
                )
        except Exception as e:
            print(f"Error updating step state: {e}")
            raise e

    def update_all_task_step_state(self, task_id: str, state: RB_Flow_Manager_ProgramState) -> None:
        try:
            if self._zenoh_client is not None:
                req = Request_Update_All_Step_StateT()
                req.taskId = task_id
                req.state = state

                self._zenoh_client.publish(
                    "rrs/task/update_all_step_state",
                    flatbuffer_req_obj=req,
                    flatbuffer_buf_size=256,
                )
        except Exception as e:
            print(f"Error updating all task step state: {e}")
            raise e

    async def _publish_executor_state(self) -> None:
        while True:
            if self._zenoh_client is None:
                break

            self._zenoh_client.publish(
                "rrs/executor/state",
                payload=self._proxy_to_dict(self._state_dicts),
            )

            await asyncio.sleep(1)

    def _proxy_to_dict(self, obj):
        if isinstance(obj, dict):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, DictProxy):
            return {k: self._proxy_to_dict(v) for k, v in obj.items()}

        if isinstance(obj, ListProxy):
            return [self._proxy_to_dict(v) for v in obj]

        return obj
