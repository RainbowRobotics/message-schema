from app.socket.socket_client import socket_client
from utils.parser import to_json

from .state_module import StateService

state_service = StateService()


@socket_client.on("call_powercontrol")
async def on_call_powercontrol(data):
    res = await state_service.power_control(
        power_option=data["power_option"], sync_servo=data["sync_servo"]
    )

    return to_json(res)


@socket_client.on("call_servocontrol")
async def on_call_servocontrol(data):
    res = await state_service.servo_control(servo_option=data["servo_option"])

    return to_json(res)
