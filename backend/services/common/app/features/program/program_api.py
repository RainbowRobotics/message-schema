from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_database.mongo_db import MongoDB
from rb_flow_manager.schema import RB_Flow_Manager_ProgramState

from .program_module import ProgramService
from .program_schema import (
    Program_Base,
    Request_Create_Multiple_StepPD,
    Request_Create_ProgramPD,
    Request_Delete_StepsPD,
    Request_Program_ExecutionPD,
    Request_Tasks_ExecutionPD,
    Request_Update_ProgramPD,
    Response_Delete_Program_And_TasksPD,
    Response_Delete_StepsPD,
    Response_Get_ProgramPD,
    Response_Get_StepListPD,
    Response_Script_ExecutionPD,
    Response_Upsert_Program_And_TasksPD,
    Step_Base,
)

program_service = ProgramService()
program_router = APIRouter(tags=["Program"])


@program_router.get("/program/{program_id}", response_model=Response_Get_ProgramPD)
async def get_program_info(program_id: str, db: MongoDB):
    res = await program_service.get_program_info(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get("/programs", response_model=list[Program_Base])
async def get_program_list(
    db: MongoDB, state: RB_Flow_Manager_ProgramState | None = None, search_name: str | None = None
):
    res = await program_service.get_program_list(state=state, search_name=search_name, db=db)
    return JSONResponse(res)


@program_router.post("/program/create", response_model=Response_Upsert_Program_And_TasksPD)
async def create_program_and_tasks(request: Request_Create_ProgramPD, db: MongoDB):
    res = await program_service.create_program_and_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.put("/program/edit", response_model=Response_Upsert_Program_And_TasksPD)
async def update_program_and_tasks(request: Request_Update_ProgramPD, db: MongoDB):
    res = await program_service.update_program_and_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.delete(
    "/program/delete/{program_id}", response_model=Response_Delete_Program_And_TasksPD
)
async def delete_program(program_id: str, db: MongoDB):
    res = await program_service.delete_program(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/step/{step_id}", response_model=Step_Base)
async def get_step(step_id: str, db: MongoDB):
    res = await program_service.get_step(step_id=step_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/steps/{task_id}", response_model=Response_Get_StepListPD)
async def get_step_list(task_id: str, db: MongoDB):
    res = await program_service.get_step_list(task_id=task_id, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/upsert", response_model=Step_Base)
async def create_or_update_steps(request: Request_Create_Multiple_StepPD, db: MongoDB):
    res = await program_service.upsert_steps(request=request, db=db)
    return JSONResponse(res)


@program_router.delete("/program/steps/delete", response_model=Response_Delete_StepsPD)
async def delete_steps(request: Request_Delete_StepsPD, db: MongoDB):
    res = await program_service.delete_steps(request=request, db=db)
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
