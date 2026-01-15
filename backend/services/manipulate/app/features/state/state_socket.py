"""Manipulate State Socket"""
from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    to_json,
)

from .state_module import (
    StateService,
)
from .state_schema import Request_PowerControlPD, Request_ReferenceControlPD, Request_ServoControlPD

state_socket_router = RbSocketIORouter()
state_service = StateService()


@state_socket_router.on("{robot_model}/call_powercontrol")
async def on_call_powercontrol(data: Request_PowerControlPD, robot_model: str):
    """
    [Power Control Socket 호출 함수]

    Args:
        data: 파워 옵션 요청 데이터 (power_option, sync_servo)
        robot_model: 로봇 모델명
    """
    res = state_service.call_powercontrol(
        robot_model=robot_model,
        power_option=data.power_option,
        sync_servo=data.sync_servo,
    )

    return to_json(res)

@state_socket_router.on("{robot_model}/call_servocontrol")
async def on_call_servocontrol(data: Request_ServoControlPD, robot_model: str):
    """
    [Servo Control Socket 호출 함수]

    Args:
        data: 서보 옵션 요청 데이터 (servo_option)
        robot_model: 로봇 모델명
    """
    res = state_service.call_servocontrol(robot_model=robot_model, request=data)

    return to_json(res)

@state_socket_router.on("{robot_model}/call_referencecontrol")
async def on_call_referencecontrol(data: Request_ReferenceControlPD, robot_model: str):
    """
    [Reference Control Socket 호출 함수]

    Args:
        data: 참조 옵션 요청 데이터 (reference_option)
        robot_model: 로봇 모델명
    """
    res = state_service.call_referencecontrol(robot_model=robot_model, request=data)

    return to_json(res)
