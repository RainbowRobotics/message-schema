from fastapi import APIRouter
from fastapi.responses import JSONResponse
from flat_buffers.IPC.State_Core import State_CoreT
from rb_schemas.base import Response_ReturnValuePD
from rb_zenoh import zenoh_client
from utils.parser import t_to_dict

from .state_module import StateService
from .state_schema import (
    PowerControlRequestPD,
    PowerControlResponsePD,
    ReferenceControlRequestPD,
    ServoControlRequestPD,
    StateRequestPD,
)

state_service = StateService()

state_router = APIRouter(tags=["State"])


@state_router.get("/{robot_model}/state_core", response_model=StateRequestPD)
async def state(robot_model: str):
    topic, mv, obj, attachment = await zenoh_client.receive_one(
        f"{robot_model}/state_core",
    )

    mv_bytes = bytes(mv)
    obj = State_CoreT.InitFromPackedBuf(mv_bytes, 0)

    return JSONResponse(t_to_dict(obj))


@state_router.post("/{robot_model}/call_powercontrol", response_model=PowerControlResponsePD)
async def power_control(robot_model: str, request: PowerControlRequestPD):
    res = await state_service.power_control(
        robot_model=robot_model, power_option=request.power_option, sync_servo=request.sync_servo
    )

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)


@state_router.post("/{robot_model}/call_servocontrol", response_model=Response_ReturnValuePD)
async def servo_control(robot_model: str, request: ServoControlRequestPD):
    res = await state_service.servo_control(
        robot_model=robot_model, servo_option=request.servo_option
    )

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)


@state_router.post("/{robot_model}/call_referencecontrol", response_model=Response_ReturnValuePD)
async def reference_control(robot_model: str, request: ReferenceControlRequestPD):
    res = await state_service.reference_control(
        robot_model=robot_model, reference_option=request.reference_option
    )

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)
