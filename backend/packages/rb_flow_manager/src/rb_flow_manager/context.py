import asyncio
import time

from rb_flat_buffers.program.RB_Program_Task_Status import RB_Program_Task_StatusT
from rb_zenoh.client import ZenohClient

from .api import FlowManagerAPI
from .control import RB_Flow_Manager_ProgramState
from .model import Locator


class Ctx:
    def __init__(self, locator: Locator):
        self.api = FlowManagerAPI()
        self.zenoh_client = ZenohClient()
        self.locator = locator

        self._pause_event = asyncio.Event()
        self._pause_event.set()

        self.zenoh_client.subscribe("/change-program-status", self.on_program_status)
        self.zenoh_client.subscribe(
            "/change-task-status", self.on_program_status, flatbuffer_obj_t=RB_Program_Task_StatusT
        )

    async def on_program_status(self, topic: str, mv: memoryview, obj: dict, attachment: dict):
        status = obj["status"]

        if status == RB_Flow_Manager_ProgramState.PAUSED:
            print("⏸️ 프로그램 일시정지됨, 대기 중...")
            self._pause_event.clear()  # pause 상태 진입
        elif status == RB_Flow_Manager_ProgramState.RUNNING:
            print("▶️ 재개됨")
            self._pause_event.set()  # resume 상태
        elif status == RB_Flow_Manager_ProgramState.STOPPED:
            print("⏹️ 완전 정지됨")
            self._pause_event.set()

    async def wait_if_paused(self):
        # 현재 pause면 RUNNING 상태가 될 때까지 block
        while not self._pause_event.is_set():
            await asyncio.sleep(0.1)  # 너무 바쁘게 돌지 않도록 짧은 sleep
        # RUNNING 상태면 바로 통과

    async def on_task_status(self, topic: str, mv: memoryview, obj: dict, attachment: dict):
        task_id = obj["task_id"]
        status = obj["status"]

        if task_id in self.locator.sync_task_ids:
            self.locator.sync_task_ids.remove(task_id)

        if len(self.locator.sync_task_ids) == 0:
            if status == "PAUSED":
                time.sleep(0.2)
            status = self.api.set_task_status(task_id, "RUNNING")
        else:
            status = self.api.set_task_status(task_id, "WAITING")
