from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
)

from .io_module import (
    IoService,
)
from .io_schema import (
    Request_SideAout_GeneralPD,
    Request_SideDout_GeneralPD,
)

io_router = APIRouter(tags=["IO"])
io_service = IoService()


@io_router.post("/{robot_model}/call_side_dout", response_model=Response_ReturnValuePD)
async def side_dout(robot_model: str, request: Request_SideDout_GeneralPD):
    res = io_service.side_dout(robot_model, request.port_num, request.desired_out)
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_side_aout", response_model=Response_ReturnValuePD)
async def side_aout(robot_model: str, request: Request_SideAout_GeneralPD):
    res = io_service.side_aout(robot_model, request.port_num, request.desired_voltage)
    return JSONResponse(res)
