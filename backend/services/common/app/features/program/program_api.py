from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_database.mongo_db import MongoDB
from rb_flow_manager.schema import RB_Flow_Manager_ProgramState

from .program_module import ProgramService
from .program_schema import (
    Program_Base,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Delete_TasksPD,
    Request_Update_ProgramPD,
    Response_Delete_Program_And_FlowsPD,
    Response_Delete_TasksPD,
    Response_Get_ProgramPD,
    Response_Get_TaskListPD,
    Response_Upsert_Program_And_FlowsPD,
    Task_Base,
)

program_service = ProgramService()
program_router = APIRouter(tags=["Program"])


# @program_router.get("/task/{task_id}/status", response_model=Response_Task_StatusPD)
# async def get_task_status(robot_model: str, task_id: str):
#     res = await program_service.get_task_status(robot_model=robot_model, task_id=task_id)
#     return res


# @program_router.post("/task/{task_id}/status", response_model=Response_ReturnValuePD)
# async def set_task_status(task_id: str, request: Request_Set_Task_StatusPD):
#     return await program_service.(task_id=task_id, request=request)


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


@program_router.post("/program/create", response_model=Response_Upsert_Program_And_FlowsPD)
async def create_program_and_flows(request: Request_Create_ProgramPD, db: MongoDB):
    res = await program_service.create_program_and_flows(request=request, db=db)
    return JSONResponse(res)


@program_router.put("/program/edit", response_model=Response_Upsert_Program_And_FlowsPD)
async def update_program_and_flows(request: Request_Update_ProgramPD, db: MongoDB):
    res = await program_service.update_program_and_flows(request=request, db=db)
    return JSONResponse(res)


@program_router.delete(
    "/program/delete/{program_id}", response_model=Response_Delete_Program_And_FlowsPD
)
async def delete_program_and_flows(program_id: str, db: MongoDB):
    res = await program_service.delete_program(program_id=program_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/task/{task_id}", response_model=Task_Base)
async def get_task(task_id: str, db: MongoDB):
    res = await program_service.get_task(task_id=task_id, db=db)
    return JSONResponse(res)


@program_router.get("/program/tasks/{flow_id}", response_model=Response_Get_TaskListPD)
async def get_task_list(flow_id: str, db: MongoDB):
    res = await program_service.get_task_list(flow_id=flow_id, db=db)
    return JSONResponse(res)


@program_router.post("/program/tasks/upsert", response_model=Task_Base)
async def create_or_update_tasks(request: Request_Create_Multiple_TaskPD, db: MongoDB):
    res = await program_service.upsert_tasks(request=request, db=db)
    return JSONResponse(res)


@program_router.delete("/program/tasks/delete", response_model=Response_Delete_TasksPD)
async def delete_tasks(request: Request_Delete_TasksPD, db: MongoDB):
    res = await program_service.delete_tasks(request=request, db=db)
    return JSONResponse(res)
