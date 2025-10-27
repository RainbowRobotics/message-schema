from typing import Annotated

from fastapi import APIRouter, Query
from rb_schemas.base import Response_ReturnValuePD
from rb_zenoh.exeption import JSONResponse

from .state_module import StateService
from .state_schema import (
    AllSwConnectStateResponsePD,
    PowerControlRequestPD,
    ServoControlRequestPD,
)

state_service = StateService()

state_router = APIRouter(tags=["State"])


@state_router.get("/{robot_model}/state_core")
async def get_state_core(robot_model: str):
    """
    Get the state of the core of the robot.
    """
    res = await state_service.get_api_state_core(robot_model)

    return JSONResponse(res)


@state_router.get("/system_state", response_model=AllSwConnectStateResponsePD)
async def system_state(
    namespaces: Annotated[list[str], Query(alias="namespaces[]")],
):
    """
    Get the state of the system.
    """
    res = await state_service.get_system_state(namespaces)

    return JSONResponse(res)


@state_router.post("/call_powercontrol", response_model=Response_ReturnValuePD)
async def power_control(request: PowerControlRequestPD):
    """
    Control the power of the robot.
    """
    res = await state_service.power_control(
        power_option=request.power_option, sync_servo=request.sync_servo or True
    )

    return JSONResponse(res)


@state_router.post("/call_servocontrol", response_model=Response_ReturnValuePD)
async def servo_control(request: ServoControlRequestPD):
    """
    Control the servo of the robot.
    """
    res = await state_service.servo_control(servo_option=request.servo_option)

    return JSONResponse(res)
