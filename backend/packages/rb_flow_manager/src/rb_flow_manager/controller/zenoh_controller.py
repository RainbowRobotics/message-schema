import time

from rb_flat_buffers.flow_manager.RB_Flow_Manager_ProgramState import RB_Flow_Manager_ProgramState
from rb_flat_buffers.flow_manager.Request_Update_Step_State import Request_Update_Step_StateT
from rb_flow_manager.controller.base_controller import BaseController
from rb_zenoh.client import ZenohClient


class Zenoh_Controller(BaseController):
    def __init__(self):
        self._zenoh_client: ZenohClient | None = None

    def on_init(self):
        self._zenoh_client = ZenohClient()

        if self._zenoh_client is None:
            raise RuntimeError("Zenoh client not initialized")

        start_time = time.time()

        while self._zenoh_client.session is None:
            if time.time() - start_time > 3.0:
                raise RuntimeError("Zenoh session not established within 3 seconds.")
            time.sleep(0.1)

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
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.ERROR)

    def on_complete(self, process_id: str, step_id: str) -> None:
        self.update_step_state(step_id, RB_Flow_Manager_ProgramState.COMPLETED)

    def on_close(self) -> None:
        if self._zenoh_client is not None and not self._zenoh_client.is_current_process_client():
            self._zenoh_client.close()
            self._zenoh_client = None

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
