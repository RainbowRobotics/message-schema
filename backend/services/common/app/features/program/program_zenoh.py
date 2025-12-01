from rb_database import get_db
from rb_flat_buffers.flow_manager.RB_Flow_Manager_ProgramState import RB_Flow_Manager_ProgramState
from rb_flat_buffers.flow_manager.Request_Update_All_Step_State import (
    Request_Update_All_Step_StateT,
)
from rb_flat_buffers.flow_manager.Request_Update_Executor_State import (
    Request_Update_Executor_StateT,
)
from rb_flat_buffers.flow_manager.Request_Update_Step_State import Request_Update_Step_StateT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from .program_module import ProgramService
from .program_schema import RB_Flow_Manager_ProgramState as RB_Flow_Manager_ProgramState_PB
from .program_schema import Request_Update_StepStatePD

zenoh_program_router = ZenohRouter()

program_service = ProgramService()


@zenoh_program_router.subscribe("rrs/pause", opts=SubscribeOptions(allowed_same_sender=True))
async def on_pause(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.call_resume_or_pause(db=db, is_pause=True)


@zenoh_program_router.subscribe("rrs/resume", opts=SubscribeOptions(allowed_same_sender=True))
async def on_resume(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.call_resume_or_pause(db=db, is_pause=False)


@zenoh_program_router.subscribe("rrs/stop", opts=SubscribeOptions(allowed_same_sender=True))
async def on_stop(*, topic, mv, obj, attachment):
    return program_service.call_stop()


@zenoh_program_router.subscribe(
    "rrs/task/update_all_step_state",
    flatbuffer_obj_t=Request_Update_All_Step_StateT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_update_all_step_state(*, topic, mv, obj, attachment):
    db = await get_db()
    task_id = obj["taskId"]
    state = convert_state_to_string(obj["state"])
    return await program_service.update_all_task_step_state(task_id=task_id, state=state, db=db)


@zenoh_program_router.subscribe(
    "rrs/step/update_state",
    flatbuffer_obj_t=Request_Update_Step_StateT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_update_step_state(*, topic, mv, obj, attachment):
    db = await get_db()
    step_id = obj["stepId"]
    task_id = obj["taskId"]
    state = convert_state_to_string(obj["state"])
    error_value: str | None = obj.get("error")
    return await program_service.update_step_state(
        request=Request_Update_StepStatePD(
            stepId=step_id, taskId=task_id, state=state, error=error_value
        ),
        db=db,
    )


def convert_state_to_string(state: RB_Flow_Manager_ProgramState) -> RB_Flow_Manager_ProgramState_PB:
    return {
        RB_Flow_Manager_ProgramState.IDLE: RB_Flow_Manager_ProgramState_PB.IDLE,
        RB_Flow_Manager_ProgramState.RUNNING: RB_Flow_Manager_ProgramState_PB.RUNNING,
        RB_Flow_Manager_ProgramState.PAUSED: RB_Flow_Manager_ProgramState_PB.PAUSED,
        RB_Flow_Manager_ProgramState.STOPPED: RB_Flow_Manager_ProgramState_PB.STOPPED,
        RB_Flow_Manager_ProgramState.WAITING: RB_Flow_Manager_ProgramState_PB.WAITING,
        RB_Flow_Manager_ProgramState.ERROR: RB_Flow_Manager_ProgramState_PB.ERROR,
        RB_Flow_Manager_ProgramState.COMPLETED: RB_Flow_Manager_ProgramState_PB.COMPLETED,
    }[state]


@zenoh_program_router.subscribe(
    "rrs/executor/state",
    flatbuffer_obj_t=Request_Update_Executor_StateT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_executor_state(*, topic, mv, obj, attachment):
    error_value: str | None = obj.get("error")
    program_service.update_executor_state(state=obj["state"], error=error_value)
