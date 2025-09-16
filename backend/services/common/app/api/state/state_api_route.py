from app.api.api_schema import BaseControlResponsePD
from app.api.state.state_api_schema import PowerControlRequestPD, ServoControlRequestPD
from app.modules.state.state_module_service import StateService
from fastapi import APIRouter
from rb_zenoh.exeption import JSONResponse

state_service = StateService()

state_router = APIRouter()


@state_router.post("/call_powercontrol", response_model=BaseControlResponsePD)
async def power_control(request: PowerControlRequestPD):
    res = await state_service.power_control(
        power_option=request.power_option, sync_servo=request.sync_servo
    )

    return JSONResponse(res)


@state_router.post("/call_servocontrol", response_model=BaseControlResponsePD)
async def servo_control(request: ServoControlRequestPD):
    res = await state_service.servo_control(servo_option=request.servo_option)

    return JSONResponse(res)
