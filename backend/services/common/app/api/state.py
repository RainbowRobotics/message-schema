from typing import Annotated

from fastapi import APIRouter, Query
from rb_zenoh.exeption import JSONResponse

from app.modules.schema import Response_ReturnValuePD
from app.modules.state.state_module_schema import (
    AllSwConnectStateResponsePD,
    PowerControlRequestPD,
    ServoControlRequestPD,
)
from app.modules.state.state_module_service import StateService

state_service = StateService()

state_router = APIRouter(tags=["State"])


@state_router.get("/system_state", response_model=AllSwConnectStateResponsePD)
async def system_state(
    namespaces: Annotated[list[str], Query(alias="namespaces[]")],
):
    res = await state_service.get_system_state(namespaces)

    return JSONResponse(res)


@state_router.post("/call_powercontrol", response_model=Response_ReturnValuePD)
async def power_control(request: PowerControlRequestPD):
    res = await state_service.power_control(
        power_option=request.power_option, sync_servo=request.sync_servo
    )

    return JSONResponse(res)


@state_router.post("/call_servocontrol", response_model=Response_ReturnValuePD)
async def servo_control(request: ServoControlRequestPD):
    res = await state_service.servo_control(servo_option=request.servo_option)

    return JSONResponse(res)
