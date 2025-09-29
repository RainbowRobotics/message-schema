from typing import Annotated

from fastapi import APIRouter, Query
from rb_schemas.base import Response_ReturnValuePD

from .program_module import ProgramService
from .program_schema import Request_Change_SpeedbarPD, Response_SpeedBarPD

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
