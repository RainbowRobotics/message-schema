from utils.parser import to_json

from app.modules.state.state_module_service import StateService
from app.socket import sio

state_service = StateService()


@sio.on("call_powercontrol")
async def on_call_powercontrol(data):
    res = await state_service.power_control(
        power_option=data["power_option"], sync_servo=data["sync_servo"]
    )

    return to_json(res)


@sio.on("call_servocontrol")
async def on_call_servocontrol(data):
    res = await state_service.servo_control(servo_option=data["servo_option"])

    return to_json(res)
