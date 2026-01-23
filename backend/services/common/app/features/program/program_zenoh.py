from typing import Literal

from rb_database import get_db
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
from rb_flat_buffers.program.RB_Program_Dialog import RB_Program_DialogT
from rb_flat_buffers.program.RB_Program_Log import RB_Program_LogT
from rb_flat_buffers.program.RB_Program_Log_Type import RB_Program_Log_Type
from rb_flat_buffers.program.Request_Exec_Control_Program import Request_Exec_Control_ProgramT
from rb_flat_buffers.program.Request_Program_At_End import Request_Program_At_EndT
from rb_flat_buffers.program.Request_Program_At_Start import Request_Program_At_StartT
from rb_flat_buffers.program.Request_Update_Sub_Task_State import Request_Update_Sub_Task_StateT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from .program_module import ProgramService
from .program_schema import RB_Flow_Manager_ProgramState as RB_Flow_Manager_ProgramState_PB
from .program_schema import Request_Update_StepStatePD

zenoh_program_router = ZenohRouter()

program_service = ProgramService()

@zenoh_program_router.subscribe("rrs/program/at_start", flatbuffer_obj_t=Request_Program_At_StartT, opts=SubscribeOptions(allowed_same_sender=True))
async def on_program_at_start(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.at_program_start(task_id=obj["taskId"], db=db)

@zenoh_program_router.subscribe("rrs/program/at_end", flatbuffer_obj_t=Request_Program_At_EndT, opts=SubscribeOptions(allowed_same_sender=True))
async def on_program_at_end(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.at_program_end(task_id=obj["taskId"], db=db)

@zenoh_program_router.queryable("rrs/pause", flatbuffer_req_t=Request_Exec_Control_ProgramT, flatbuffer_res_buf_size=8)
async def on_pause():
    """ 전체 로봇 프로그램 일시정지 """
    return program_service.call_resume_or_pause(is_pause=True)


@zenoh_program_router.queryable("rrs/resume")
def on_resume():
    """ 전체 로봇 프로그램 재개 """
    return program_service.call_resume_or_pause(is_pause=False)


@zenoh_program_router.queryable("rrs/stop")
async def on_stop():
    """ 전체 로봇 프로그램 중지 """
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
    "rrs/program/*/sub_task/update_state",
    flatbuffer_obj_t=Request_Update_Sub_Task_StateT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_update_sub_task_state(*, topic, mv, obj, attachment):
    task_id = topic.split("/")[2]

    program_service.update_sub_task_state(task_id=task_id)

@zenoh_program_router.subscribe(
    "rrs/executor/state",
    flatbuffer_obj_t=Request_Update_Executor_StateT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_executor_state(*, topic, mv, obj, attachment):
    error_value: str | None = obj.get("error")
    program_service.update_executor_state(state=obj["state"], error=error_value)

@zenoh_program_router.subscribe(
    "rrs/program/dialog",
    flatbuffer_obj_t=RB_Program_DialogT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_program_dialog(*, topic, mv, obj, attachment):
    program_service.program_dialog(obj)

@zenoh_program_router.subscribe(
    "rrs/program/log",
    flatbuffer_obj_t=RB_Program_LogT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_program_log(*, topic, mv, obj, attachment):
    log_type = obj.get("type", None)
    log_level: Literal["INFO", "WARNING", "ERROR", "USER", "DEBUG", "GENERAL"] = "DEBUG"

    if log_type == RB_Program_Log_Type.INFO:
        log_level = "INFO"
    elif log_type == RB_Program_Log_Type.WARNING:
        log_level = "WARNING"
    elif log_type == RB_Program_Log_Type.ERROR:
        log_level = "ERROR"
    elif log_type == RB_Program_Log_Type.USER:
        log_level = "USER"
    elif log_type == RB_Program_Log_Type.DEBUG:
        log_level = "DEBUG"
    elif log_type == RB_Program_Log_Type.GENERAL:
        log_level = "GENERAL"

    program_service.program_log(request={
        "content": obj["content"],
        "robot_model": obj["robotModel"],
        "level": log_level,
    })
