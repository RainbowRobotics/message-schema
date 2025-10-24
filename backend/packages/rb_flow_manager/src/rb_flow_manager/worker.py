from rb_flat_buffers.program.RB_Program_Task_Status import RB_Program_Task_StatusT
from rb_zenoh.client import ZenohClient

from .control import ControlFlags, RB_Flow_Manager_ProgramState
from .tree import Node, Step, TreeRunner


def build_tree(component: str) -> Node:
    return Node(
        component,
        children=[
            Step("Repeat", lambda ctx: [ctx.ctrl.raise_if_stopped(), ctx.ctrl.wait_if_paused()]),
            Step(
                "CustomLambda",
                lambda ctx: print(f"[{ctx.component}] user lambda, off={ctx.offset}"),
            ),
        ],
    )


def start_worker(
    program_id: str,
    task_id: str,
    tree: Node,
):
    zenoh_client = ZenohClient()
    ctrl = ControlFlags()

    def on_change_task_status(topic, mv, obj, attachment):
        event_task_id = obj.get("task_id")
        event_status = obj.get("status")

        if (
            task_id == event_task_id
            and event_status in RB_Flow_Manager_ProgramState._value2member_map_
        ):
            ctrl.set_state(RB_Flow_Manager_ProgramState(event_status))
            print(f"[{task_id}] ◀ control: {event_status}")

    # zen.subscribe("/control/all", on_control)
    zenoh_client.subscribe(
        "/change-task-status", on_change_task_status, flatbuffer_obj_t=RB_Program_Task_StatusT
    )

    def publish(
        status: str,
    ):
        req = RB_Program_Task_StatusT()
        req.status = status
        req.taskId = task_id

        req.programId = program_id
        req.syncTaskIds = []
        req.nodePath = []
        req.offset = 0

        zenoh_client.publish(
            "/change-task-status",
            flatbuffer_req_obj=req,
        )

    try:
        TreeRunner(task_id=task_id, program_id=program_id, ctrl=ctrl).run(tree)

        publish(RB_Flow_Manager_ProgramState.COMPLETED)
    except StopIteration:
        publish(RB_Flow_Manager_ProgramState.STOPPED)
    except Exception:
        publish(RB_Flow_Manager_ProgramState.ERROR)
