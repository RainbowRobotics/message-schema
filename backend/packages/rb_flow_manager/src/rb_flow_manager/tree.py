# tree.py
import threading
import uuid
from collections.abc import Callable
from typing import Union

from rb_flat_buffers.program.RB_Program_Task_Checkpoint import RB_Program_Task_CheckpointT
from rb_flat_buffers.program.RB_Program_Task_Status import RB_Program_Task_StatusT
from rb_modules.log import zenoh_client
from rb_zenoh.client import ZenohClient

from .control import ControlFlags, RB_Flow_Manager_ProgramState


class Ctx:
    def __init__(self, ctrl: ControlFlags):
        self.ctrl = ctrl
        self.sync_step_ids = []
        self.step_info = {}

        self.zenoh_client = ZenohClient()

        self.zenoh_client.subscribe("/change-task-checkpoint", self.on_change_task_checkpoint)

    def set_sync_step_ids(self, sync_step_ids: list[str]):
        self.sync_step_ids = sync_step_ids

    def on_change_task_checkpoint(self, topic: str, mv: memoryview, obj: dict, attachment: dict):
        task_id = obj["task_id"]
        step_id = obj["step_id"]
        status = obj["status"]

        self.step_info[task_id] = {
            "step_id": step_id,
            "status": status,
        }

        for sync_step_id in self.sync_step_ids:
            for step_info in self.step_info.values():
                if (
                    step_info["step_id"] == sync_step_id
                    and step_info["status"] == RB_Flow_Manager_ProgramState.WAITING
                ):
                    self.sync_step_ids.remove(sync_step_id)
                    break

        if len(self.sync_step_ids) == 0 and self.ctrl.is_paused():
            req = RB_Program_Task_StatusT()
            req.status = RB_Flow_Manager_ProgramState.RUNNING
            req.taskId = self.task_id
            req.programId = self.program_id
            req.syncTaskIds = []
            req.nodePath = []
            req.offset = 0

            self.zenoh_client.publish("/change-task-status", flatbuffer_req_obj=req)
        elif len(self.sync_step_ids) > 0:
            req = RB_Program_Task_StatusT()
            req.status = RB_Flow_Manager_ProgramState.WAITING
            req.taskId = self.task_id
            req.programId = self.program_id
            req.syncTaskIds = []
            req.nodePath = []
            req.offset = 0

            self.zenoh_client.publish("/change-task-status", flatbuffer_req_obj=req)


class Step:
    def __init__(
        self, name: str, fn: Callable[[Ctx], None], *, sync_step_ids: list[str] | None = None
    ):
        self.step_id = self._make_step_id()
        self.name = name
        self.fn = fn
        self.sync_step_ids = sync_step_ids or []

    def _make_step_id(self):
        return f"{self.name}-{uuid.uuid4()}"

    def run(self, ctx: Ctx):
        ctx.ctrl.raise_if_stopped()
        ctx.ctrl.wait_if_paused()

        done_evt = threading.Event()

        def done():
            done_evt.set()

        try:
            if self._requires_done():
                self.fn(ctx, done)
                done_evt.wait()
            else:
                self.fn(ctx)
        except Exception as e:
            print(f"[{ctx.component}] ⚠️ Step '{self.name}' error: {e}")
            raise e

    def _requires_done(self) -> bool:
        import inspect

        params = inspect.signature(self.fn).parameters
        return "done" in params


class Node:
    def __init__(self, name: str, children: list[Union["Node", Step]]):
        self.name, self.children = name, children


class TreeRunner:
    def __init__(self, task_id: str, program_id: str, ctrl: ControlFlags):
        self.task_id = task_id
        self.program_id = program_id
        self.ctrl = ctrl

    def run(self, root: Node):
        ctx = Ctx(self.task_id, self.program_id, self.ctrl)
        self._dfs(root, ctx)

    def _dfs(self, node: Node, ctx: Ctx):
        req = RB_Program_Task_CheckpointT()
        req.taskId = self.task_id
        req.programId = self.program_id
        req.syncTaskIds = []
        req.nodePath = []
        req.offset = 0
        req.status = RB_Flow_Manager_ProgramState.RUNNING

        for child in node.children:
            req.syncTaskIds = []
            # req.offset = child.offset

            if isinstance(child, Step):
                print(f"[{ctx.component}] ▶ Step start: {child.name}")
                req.syncTaskIds = child.sync_task_ids
                zenoh_client.publish("change-task-checkpoint", flatbuffer_req_obj=req)

                child.run(ctx)
                print(f"[{ctx.component}] ✅ Step done : {child.name}")
            else:
                req.nodePath = req.node_path.append(child.name)
                zenoh_client.publish("change-task-checkpoint", flatbuffer_req_obj=req)

                self._dfs(child, ctx)
