from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)
from rb_flat_buffers.IPC.State_Core import (
    State_CoreT,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
)
from rb_zenoh.client import (
    ZenohClient,
)

from .state_module import (
    StateService,
)
from .state_schema import (
    Request_PowerControlPD,
    Request_ReferenceControlPD,
    Request_ServoControlPD,
    Response_PowerControlPD,
    StateRequestPD,
)

state_service = StateService()
zenoh_client = ZenohClient()
state_router = APIRouter(tags=["State"])


@state_router.get("/{robot_model}/state_core", response_model=StateRequestPD)
async def state_core(robot_model: str):
    topic, mv, obj, attachment = await zenoh_client.receive_one(
        f"{robot_model}/state_core",
        flatbuffer_obj_t=State_CoreT,
    )
    return JSONResponse(obj)


@state_router.post("/{robot_model}/call_powercontrol", response_model=Response_PowerControlPD)
async def call_powercontrol(robot_model: str, request: Request_PowerControlPD):
    res = await state_service.call_powercontrol(
        robot_model=robot_model,
        power_option=request.power_option,
        sync_servo=request.sync_servo if request.sync_servo is not None else True,
    )

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)


@state_router.post("/{robot_model}/call_servocontrol", response_model=Response_ReturnValuePD)
async def call_servocontrol(robot_model: str, request: Request_ServoControlPD):
    res = await state_service.call_servocontrol(robot_model=robot_model, request=request)

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)


@state_router.post("/{robot_model}/call_referencecontrol", response_model=Response_ReturnValuePD)
async def call_referencecontrol(robot_model: str, request: Request_ReferenceControlPD):
    res = await state_service.call_referencecontrol(robot_model=robot_model, request=request)

    if res.get("error"):
        return JSONResponse(content=res, status_code=500)

    return JSONResponse(res)
