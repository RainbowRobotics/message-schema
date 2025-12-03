from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_schemas.base import Response_ReturnValuePD

from .io_module import IoService
from .io_schema import (
    Request_Flange_Digital_OutPD,
    Request_Flange_PowerPD,
    Request_SideAout_GeneralPD,
    Request_SideDout_BitcombinationPD,
    Request_SideDout_GeneralPD,
    Request_SideDout_PulsePD,
    Request_SideDout_TogglePD,
)

io_router = APIRouter(tags=["IO"])
io_service = IoService()



@io_router.post("/{robot_model}/call_side_dout", response_model=Response_ReturnValuePD)
async def call_side_dout(robot_model: str, request: Request_SideDout_GeneralPD):
    res = await io_service.call_side_dout(robot_model, request.port_num, request.desired_out)
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_side_aout", response_model=Response_ReturnValuePD)
async def call_side_aout(robot_model: str, request: Request_SideAout_GeneralPD):
    res = await io_service.call_side_aout(robot_model, request.port_num, request.desired_voltage)
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_flange_power", response_model=Response_ReturnValuePD)
async def call_flange_power(robot_model: str, request: Request_Flange_PowerPD):
    res = await io_service.call_flange_power(robot_model, request.desired_voltage)
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_flange_dout", response_model=Response_ReturnValuePD)
async def call_flange_dout(robot_model: str, request: Request_Flange_Digital_OutPD):
    res = await io_service.call_flange_dout(robot_model, request.port_num, request.desired_out)
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_side_dout_toggle", response_model=Response_ReturnValuePD)
async def call_side_dout_toggle(robot_model: str, request: Request_SideDout_TogglePD):
    res = await io_service.call_side_dout_toggle(robot_model, request.port_num)
    return JSONResponse(res)


@io_router.post(
    "/{robot_model}/call_side_dout_bitcombination", response_model=Response_ReturnValuePD
)
async def call_side_dout_bitcombination(robot_model: str, request: Request_SideDout_BitcombinationPD):
    res = await io_service.call_side_dout_bitcombination(
        robot_model,
        request.port_start,
        request.port_end,
        request.desired_value,
        request.direction_option,
    )
    return JSONResponse(res)


@io_router.post("/{robot_model}/call_side_dout_pulse", response_model=Response_ReturnValuePD)
async def call_side_dout_pulse(robot_model: str, request: Request_SideDout_PulsePD):
    res = await io_service.call_side_dout_pulse(
        robot_model,
        request.port_num,
        request.block_mode,
        request.direction,
        request.time_1,
        request.time_2,
        request.time_3,
    )
    return JSONResponse(res)
