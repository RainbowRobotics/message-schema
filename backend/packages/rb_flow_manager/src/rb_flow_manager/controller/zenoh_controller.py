import time

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
        pass

    def on_stop(self, process_id: str, step_id: str) -> None:
        pass

    def on_pause(self, process_id: str, step_id: str) -> None:
        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/pause", payload={})

    def on_resume(self, process_id: str, step_id: str) -> None:
        if self._zenoh_client is not None:
            self._zenoh_client.publish("rrs/resume", payload={})

    def on_next(self, process_id: str, step_id: str) -> None:
        pass

    def on_error(self, process_id: str, step_id: str, error: Exception) -> None:
        pass

    def on_complete(self, process_id: str) -> None:
        pass

    def on_close(self) -> None:
        if self._zenoh_client is not None:
            self._zenoh_client.close()
            self._zenoh_client = None

    def on_all_complete(self) -> None:
        pass

    def on_all_stop(self) -> None:
        pass

    def on_all_pause(self) -> None:
        pass
