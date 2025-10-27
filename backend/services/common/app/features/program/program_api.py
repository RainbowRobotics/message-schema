from typing import Annotated

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse
from rb_database.mongo_db import MongoDB
from rb_schemas.base import Response_ReturnValuePD

from .program_module import ProgramService
from .program_schema import (
    Request_Change_SpeedbarPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Set_Task_StatusPD,
    Request_Update_ProgramPD,
    Response_Delete_Program_And_FlowsPD,
    Response_SpeedBarPD,
    Response_Task_StatusPD,
    Response_Upsert_Program_And_FlowsPD,
    Task_Base,
)

program_service = ProgramService()
program_router = APIRouter(tags=["Program"])


@program_router.get("/speedbar", response_model=Response_SpeedBarPD)
async def get_speedbar(
    components: Annotated[list[str], Query(alias="components[]")],
):
    res = await program_service.get_all_speedbar(components=components)

    return res


@program_router.post("/speedbar", response_model=Response_ReturnValuePD)
async def change_speedbar(request: Request_Change_SpeedbarPD):
    return await program_service.control_speed_bar(**request.model_dump())


@program_router.get("/task/{task_id}/status", response_model=Response_Task_StatusPD)
async def get_task_status(robot_model: str, task_id: str):
    res = await program_service.get_task_status(robot_model=robot_model, task_id=task_id)
    return res


@program_router.post("/task/{task_id}/status", response_model=Response_ReturnValuePD)
async def set_task_status(task_id: str, request: Request_Set_Task_StatusPD):
    return await program_service.set_task_status(task_id=task_id, request=request)


@program_router.post("/program", response_model=Response_Upsert_Program_And_FlowsPD)
async def create_program_and_flows(request: Request_Create_ProgramPD, db: MongoDB):
    res = await program_service.create_program_and_flows(request=request, db=db)
    return JSONResponse(res)


@program_router.put("/program", response_model=Response_Upsert_Program_And_FlowsPD)
async def update_program_and_flows(request: Request_Update_ProgramPD, db: MongoDB):
    res = await program_service.update_program_and_flows(request=request, db=db)
    return JSONResponse(res)


@program_router.delete("/program/{program_id}", response_model=Response_Delete_Program_And_FlowsPD)
async def delete_program_and_flows(program_id: str, db: MongoDB):
    res = await program_service.delete_program(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks", response_model=Task_Base)
async def create_or_update_tasks(request: Request_Create_Multiple_TaskPD, db: MongoDB):
    res = await program_service.upsert_tasks(request=request, db=db)
    return JSONResponse(res)
