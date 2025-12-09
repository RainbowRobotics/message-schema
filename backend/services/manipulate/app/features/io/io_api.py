from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
)

from .io_module import IoService
from .io_schema import (
    Request_Flange_Digital_OutPD,
    Request_Flange_PowerPD,
    Request_Multiple_SideDoutBitcombinationPD,
    Request_Multiple_SideDoutPD,
    Request_Multiple_SideDoutPulsePD,
    Request_Multiple_SideDoutTogglePD,
    Request_Save_SideDin_FilterCountPD,
    Request_Save_SideDin_SpecialFuncPD,
    Request_Save_SideDout_SpecialFuncPD,
    Request_SideAout_GeneralPD,
    Request_SideDout_BitcombinationPD,
    Request_SideDout_GeneralPD,
    Request_SideDout_PulsePD,
    Request_SideDout_TogglePD,
)

io_router = APIRouter(tags=["IO"])
io_service = IoService()


# ==============================================================================
# 1. Side IO Control
# ==============================================================================

@io_router.post("/{robot_model}/call_side_dout", response_model=Response_ReturnValuePD)
async def call_side_dout(robot_model: str, request: Request_SideDout_GeneralPD):
    return JSONResponse(io_service.call_side_dout(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/call_side_aout", response_model=Response_ReturnValuePD)
async def call_side_aout(robot_model: str, request: Request_SideAout_GeneralPD):
    return JSONResponse(io_service.call_side_aout(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/call_side_dout_toggle", response_model=Response_ReturnValuePD)
async def call_side_dout_toggle(robot_model: str, request: Request_SideDout_TogglePD):
    return JSONResponse(io_service.call_side_dout_toggle(robot_model=robot_model, request=request))


@io_router.post(
    "/{robot_model}/call_side_dout_bitcombination", response_model=Response_ReturnValuePD
)
async def call_side_dout_bitcombination(robot_model: str, request: Request_SideDout_BitcombinationPD):
    return JSONResponse(io_service.call_side_dout_bitcombination(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/call_side_dout_pulse", response_model=Response_ReturnValuePD)
async def call_side_dout_pulse(robot_model: str, request: Request_SideDout_PulsePD):
    return JSONResponse(io_service.call_side_dout_pulse(robot_model=robot_model, request=request))

@io_router.post("/{robot_model}/call_multiple_side_dout", response_model=Response_ReturnValuePD)
async def call_multiple_side_dout(robot_model: str, request: Request_Multiple_SideDoutPD):
    return JSONResponse(io_service.call_multiple_side_dout(robot_model=robot_model, request=request))

@io_router.post("/{robot_model}/call_multiple_side_dout_toggle", response_model=Response_ReturnValuePD)
async def call_multiple_side_dout_toggle(robot_model: str, request: Request_Multiple_SideDoutTogglePD):
    return JSONResponse(io_service.call_multiple_side_dout_toggle(robot_model=robot_model, request=request))

@io_router.post("/{robot_model}/call_multiple_side_dout_bitcombination", response_model=Response_ReturnValuePD)
async def call_multiple_side_dout_bitcombination(robot_model: str, request: Request_Multiple_SideDoutBitcombinationPD):
    return JSONResponse(io_service.call_multiple_side_dout_bitcombination(robot_model=robot_model, request=request))

@io_router.post("/{robot_model}/call_multiple_side_dout_pulse", response_model=Response_ReturnValuePD)
async def call_multiple_side_dout_pulse(robot_model: str, request: Request_Multiple_SideDoutPulsePD):
    return JSONResponse(io_service.call_multiple_side_dout_pulse(robot_model=robot_model, request=request))

# ==============================================================================
# 2. IO Configuration (DIN/DOUT)
# ==============================================================================

@io_router.post("/{robot_model}/save_side_dout_function", response_model=Response_ReturnValuePD)
async def save_side_dout_function(robot_model: str, request: Request_Save_SideDout_SpecialFuncPD):
    return JSONResponse(await io_service.save_side_dout_function(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/save_side_din_function", response_model=Response_ReturnValuePD)
async def save_side_din_function(robot_model: str, request: Request_Save_SideDin_SpecialFuncPD):
    return JSONResponse(await io_service.save_side_din_function(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/save_side_din_filter", response_model=Response_ReturnValuePD)
async def save_side_din_filter(robot_model: str, request: Request_Save_SideDin_FilterCountPD):
    return JSONResponse(await io_service.save_side_din_filter(robot_model=robot_model, request=request))


# ==============================================================================
# 3. Flange IO Control
# ==============================================================================

@io_router.post("/{robot_model}/call_flange_power", response_model=Response_ReturnValuePD)
async def call_flange_power(robot_model: str, request: Request_Flange_PowerPD):
    return JSONResponse(await io_service.call_flange_power(robot_model=robot_model, request=request))


@io_router.post("/{robot_model}/call_flange_dout", response_model=Response_ReturnValuePD)
async def call_flange_dout(robot_model: str, request: Request_Flange_Digital_OutPD):
    return JSONResponse(await io_service.call_flange_dout(robot_model=robot_model, request=request))
