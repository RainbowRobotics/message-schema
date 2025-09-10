from app.modules.state.state_module_service import StateService
from fastapi import APIRouter

from .state_api_schema import (
    PowerControlRequestPD,
    ReferenceControlRequestPD,
    ServoControlRequestPD,
)

state_service = StateService()

state_router = APIRouter()


@state_router.post("/{robot_model}/state/power")
def power_control(robot_model: str, request: PowerControlRequestPD):
    return state_service.power_control(
        robot_model=robot_model, power_option=request.power_option, sync_servo=request.sync_servo
    )


@state_router.post("/{robot_model}/state/servo")
def servo_control(robot_model: str, request: ServoControlRequestPD):
    return state_service.servo_control(robot_model=robot_model, servo_option=request.servo_option)


@state_router.post("/{robot_model}/state/reference")
def reference_control(robot_model: str, request: ReferenceControlRequestPD):
    return state_service.reference_control(
        robot_model=robot_model, reference_option=request.reference_option
    )
