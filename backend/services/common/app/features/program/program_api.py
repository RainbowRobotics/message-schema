import inspect
from typing import Literal

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_database.mongo_db import MongoDB
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.program.Request_Exec_Control_Program import Request_Exec_Control_ProgramT
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from .program_module import (
    ProgramService,
)
from .program_schema import (
    Request_Clone_ProgramPD,
    Request_Create_Multiple_StepPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Delete_StepsPD,
    Request_Get_Script_ContextPD,
    Request_Load_ProgramPD,
    Request_Preview_Reset_ProgramPD,
    Request_Preview_Start_ProgramPD,
    Request_Preview_Stop_ProgramPD,
    Request_Program_ExecutionPD,
    Request_Tasks_ExecutionPD,
    Request_Update_Multiple_TaskPD,
    Request_Update_ProgramPD,
    Response_Clone_ProgramPD,
    Response_Create_Multiple_TaskPD,
    Response_Create_Program_And_TasksPD,
    Response_Delete_Program_And_TasksPD,
    Response_Delete_StepsPD,
    Response_Get_Current_Program_StatePD,
    Response_Get_Executor_VariablesPD,
    Response_Get_Program_ListPD,
    Response_Get_ProgramPD,
    Response_Get_Script_ContextPD,
    Response_Get_StepListPD,
    Response_Get_Sub_Task_ListPD,
    Response_Get_Task_ListPD,
    Response_Get_TaskInfoPD,
    Response_Get_TaskStatePD,
    Response_Script_ExecutionPD,
    Response_Update_Multiple_TaskPD,
    Response_Update_ProgramPD,
    Response_Upsert_StepsPD,
    Step_Base,
    TaskType,
)

program_service = ProgramService()
program_router = APIRouter(tags=["Program"])
zenoh_client = ZenohClient()


@program_router.post("/program/load", response_model=Response_Get_ProgramPD)
async def load_program(request: Request_Load_ProgramPD, db: MongoDB):
    res = await program_service.load_program(request=request, db=db)
    return JSONResponse(res)


@program_router.get("/program/{program_id}", response_model=Response_Get_ProgramPD)
async def get_program_info(program_id: str, db: MongoDB):
    res = await program_service.get_program_info(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get("/programs", response_model=Response_Get_Program_ListPD)
async def get_program_list(db: MongoDB, search_name: str | None = None, order: Literal["ASC", "DESC"] | None = None):
    res = await program_service.get_program_list(search_name=search_name, order=order, db=db)
    return JSONResponse(res)


@program_router.post("/program/create", response_model=Response_Create_Program_And_TasksPD)
async def create_program_and_tasks(request: Request_Create_ProgramPD, db: MongoDB):
    res = await program_service.create_program_and_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.put("/program/edit", response_model=Response_Update_ProgramPD)
async def update_program(request: Request_Update_ProgramPD, db: MongoDB):
    res = await program_service.update_program(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/clone", response_model=Response_Clone_ProgramPD)
async def clone_program(request: Request_Clone_ProgramPD, db: MongoDB):
    res = await program_service.clone_program(request=request, db=db)
    return JSONResponse(res)


@program_router.delete(
    "/program/delete/{program_id}", response_model=Response_Delete_Program_And_TasksPD
)
async def delete_program(program_id: str, db: MongoDB):
    res = await program_service.delete_program(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/main-tasks/{program_id}", response_model=Response_Get_Task_ListPD)
async def get_main_task_list(program_id: str, db: MongoDB):
    res = await program_service.get_main_task_list(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get(
    "/program/sub-tasks/{program_id}/{parent_task_id}", response_model=Response_Get_Task_ListPD
)
async def get_sub_task_list(program_id: str, parent_task_id: str, db: MongoDB):
    res = await program_service.get_sub_task_list(
        program_id=program_id, parent_task_id=parent_task_id, db=db
    )
    return JSONResponse(res)


@program_router.get("/program/current/state", response_model=Response_Get_Current_Program_StatePD)
async def get_current_program_state():
    res = program_service.get_play_state()
    return JSONResponse(res)

@program_router.get("/program/task/list", response_model=Response_Get_Task_ListPD)
async def get_task_list(db: MongoDB, robot_model: str, task_type: TaskType | None = None, search_text: str | None = None):
    res = await program_service.get_task_list(task_type=task_type, robot_model=robot_model, search_text=search_text, db=db)
    return JSONResponse(res)

@program_router.get("/program/task/{task_id}/sub-task-list", response_model=Response_Get_Sub_Task_ListPD)
async def get_ctx_sub_task_list(task_id: str):
    res = program_service.get_ctx_sub_task_list(task_id=task_id)
    return JSONResponse(res)

@program_router.get("/program/task/{task_id}", response_model=Response_Get_TaskInfoPD)
async def get_task_info(task_id: str, db: MongoDB):
    res = await program_service.get_task_info(task_id=task_id, db=db)
    return JSONResponse(res)

@program_router.get("/program/task/{task_id}/state", response_model=Response_Get_TaskStatePD)
async def get_task_state(task_id: str):
    res = await program_service.get_task_state(task_id=task_id)
    return JSONResponse(res)

@program_router.post("/program/tasks/create", response_model=Response_Create_Multiple_TaskPD)
async def create_tasks(request: Request_Create_Multiple_TaskPD, db: MongoDB):
    res = await program_service.create_tasks(request=request, db=db)
    return JSONResponse(res)

@program_router.post("/program/tasks/update", response_model=Response_Update_Multiple_TaskPD)
async def update_tasks(request: Request_Update_Multiple_TaskPD, db: MongoDB):
    res = await program_service.update_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.get("/program/step/{step_id}", response_model=Step_Base)
async def get_step(step_id: str, db: MongoDB):
    res = await program_service.get_step(step_id=step_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/steps/{task_id}", response_model=Response_Get_StepListPD)
async def get_step_list(task_id: str, db: MongoDB):
    res = await program_service.get_step_list(task_id=task_id, db=db)
    return JSONResponse(res)


@program_router.post("/program/steps/upsert", response_model=Response_Upsert_StepsPD)
async def create_or_update_steps(request: Request_Create_Multiple_StepPD, db: MongoDB):
    res = await program_service.upsert_steps(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/task/script-context", response_model=Response_Get_Script_ContextPD)
async def get_script_context(request: Request_Get_Script_ContextPD, db: MongoDB):
    res = await program_service.get_script_context(request=request, db=db)
    return JSONResponse(res)


@program_router.delete("/program/steps/delete", response_model=Response_Delete_StepsPD)
async def delete_steps(request: Request_Delete_StepsPD, db: MongoDB):
    res = await program_service.delete_steps(request=request, db=db)
    return JSONResponse(res)


@program_router.get("/program/executor/state", response_model=dict)
async def get_executor_state():
    res = program_service.get_executor_state()
    return JSONResponse(res)

@program_router.get("/program/{robot_model}/variables", response_model=Response_Get_Executor_VariablesPD)
async def get_executor_variables(robot_model: str):
    res = program_service.get_executor_variables(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/program/preview/start", response_model=Response_Script_ExecutionPD)
async def preview_start_program(request: Request_Preview_Start_ProgramPD, db: MongoDB):
    res = await program_service.preview_start_program(request=request, db=db)
    return JSONResponse(res)

@program_router.post("/program/preview/reset", response_model=Response_Script_ExecutionPD)
async def preview_reset_program(request: Request_Preview_Reset_ProgramPD, db: MongoDB):
    res = await program_service.preview_reset_program(request=request, db=db)
    return JSONResponse(res)

@program_router.post("/program/preview/stop", response_model=Response_Script_ExecutionPD)
async def preview_stop_program(request: Request_Preview_Stop_ProgramPD):
    res = await program_service.preview_stop_program(request=request)
    return JSONResponse(res)


@program_router.post("/program/preview/pause", response_model=Response_Script_ExecutionPD)
async def preview_pause_program(request: Request_Preview_Stop_ProgramPD):
    res = await program_service.preview_pause_program(request=request)
    return JSONResponse(res)


@program_router.post("/program/preview/resume", response_model=Response_Script_ExecutionPD)
async def preview_resume_program(request: Request_Preview_Stop_ProgramPD):
    res = await program_service.preview_resume_program(request=request)
    print("TYPE:", type(res), flush=True)
    print("is coroutine?", inspect.iscoroutine(res), flush=True)
    return JSONResponse(res)


@program_router.post("/program/start", response_model=Response_Script_ExecutionPD)
async def start_program(request: Request_Program_ExecutionPD, db: MongoDB):
    res = await program_service.start_program(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/stop", response_model=Response_Script_ExecutionPD)
async def stop_program(request: Request_Program_ExecutionPD, db: MongoDB):
    res = await program_service.stop_program(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/pause", response_model=Response_Script_ExecutionPD)
async def pause_program(request: Request_Program_ExecutionPD, db: MongoDB):
    res = await program_service.pause_program(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/resume", response_model=Response_Script_ExecutionPD)
async def resume_program(request: Request_Program_ExecutionPD, db: MongoDB):
    res = await program_service.resume_program(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/start", response_model=Response_Script_ExecutionPD)
async def start_tasks(request: Request_Tasks_ExecutionPD, db: MongoDB):
    res = await program_service.start_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/stop", response_model=Response_Script_ExecutionPD)
async def stop_tasks(request: Request_Tasks_ExecutionPD, db: MongoDB):
    res = await program_service.stop_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/pause", response_model=Response_Script_ExecutionPD)
async def pause_tasks(request: Request_Tasks_ExecutionPD, db: MongoDB):
    res = await program_service.pause_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/resume", response_model=Response_Script_ExecutionPD)
async def resume_tasks(request: Request_Tasks_ExecutionPD, db: MongoDB):
    res = await program_service.resume_tasks(request=request, db=db)
    return JSONResponse(res)

@program_router.post("/test/executor/{status}", response_model=Response_Script_ExecutionPD)
async def test_executor(status: Literal["pause", "resume", "stop"]):
    if status == "pause":
        req = Request_Exec_Control_ProgramT()
        req.robotModel = "C500920"
        req.programId = "1234567890"
        res =zenoh_client.query_one("rrs/pause", flatbuffer_req_obj=req, flatbuffer_res_T_class=Response_FunctionsT, flatbuffer_buf_size=50)

        print("res >>", res, flush=True)

        return JSONResponse(t_to_dict(res))
    elif status == "resume":
        zenoh_client.query_one("rrs/resume")
    elif status == "stop":
        zenoh_client.query_one("rrs/stop")
    return JSONResponse({"status": "success"})
