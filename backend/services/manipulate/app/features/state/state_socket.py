from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)

from .state_module import (
    StateService,
)

state_socket_router = RbSocketIORouter()
state_service = StateService()


@state_socket_router.on("{robot_model}/call_powercontrol")
async def on_call_powercontrol(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = await state_service.power_control(
        robot_model=robot_model,
        power_option=dict_data["power_option"],
        sync_servo=dict_data["sync_servo"],
    )

    return to_json(res)


@state_socket_router.on("{robot_model}/call_servocontrol")
async def on_call_servocontrol(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = await state_service.servo_control(
        robot_model=robot_model,
        servo_option=dict_data["servo_option"],
    )

    return to_json(res)


@state_socket_router.on("{robot_model}/call_referencecontrol")
async def on_call_referencecontrol(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = await state_service.reference_control(
        robot_model=robot_model,
        reference_option=dict_data["reference_option"],
    )

    return to_json(res)
