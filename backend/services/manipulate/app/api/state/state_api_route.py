from app.modules.state.state_module_service import StateService
from fastapi import APIRouter
from fastapi.responses import JSONResponse

from .state_api_schema import (
    BaseControlResponsePD,
    PowerControlRequestPD,
    ReferenceControlRequestPD,
    ServoControlRequestPD,
)

state_service = StateService()

state_router = APIRouter()


@state_router.post("/{robot_model}/state/power", response_model=BaseControlResponsePD)
async def power_control(robot_model: str, request: PowerControlRequestPD):
    res = await state_service.power_control(
        robot_model=robot_model, power_option=request.power_option, sync_servo=request.sync_servo
    )
    return JSONResponse(res)


@state_router.post("/{robot_model}/state/servo", response_model=BaseControlResponsePD)
async def servo_control(robot_model: str, request: ServoControlRequestPD):
    res = await state_service.servo_control(
        robot_model=robot_model, servo_option=request.servo_option
    )
    return JSONResponse(res)


@state_router.post("/{robot_model}/state/reference", response_model=BaseControlResponsePD)
async def reference_control(robot_model: str, request: ReferenceControlRequestPD):
    res = await state_service.reference_control(
        robot_model=robot_model, reference_option=request.reference_option
    )
    return JSONResponse(res)
