from rb_socketio import RbSocketIORouter
from rb_utils.parser import t_to_dict, to_json

from .io_module import IoService

io_socket_router = RbSocketIORouter()
io_service = IoService()


@io_socket_router.on("{robot_model}/call_side_dout")
async def on_call_side_dout(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = await io_service.side_dout(robot_model, dict_data["port_num"], dict_data["desired_out"])
    return to_json(res)


@io_socket_router.on("{robot_model}/call_side_aout")
async def on_call_side_aout(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = await io_service.side_aout(
        robot_model, dict_data["port_num"], dict_data["desired_voltage"]
    )
    return to_json(res)


@io_socket_router.on("{robot_model}/call_flange_power")
async def on_call_flange_power(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = await io_service.flange_power(robot_model, dict_data["desired_voltage"])
    return to_json(res)


@io_socket_router.on("{robot_model}/call_side_dout_bitcombination")
async def on_call_side_dout_bitcombination(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = await io_service.side_dout_bitcombination(
        robot_model,
        dict_data["port_start"],
        dict_data["port_end"],
        dict_data["desired_value"],
        dict_data["direction_option"],
    )
    return to_json(res)


@io_socket_router.on("{robot_model}/call_side_dout_pulse")
async def on_call_side_dout_pulse(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = await io_service.side_dout_pulse(
        robot_model,
        dict_data["port_num"],
        dict_data["block_mode"],
        dict_data["direction"],
        dict_data["time_1"],
        dict_data["time_2"],
        dict_data["time_3"],
    )
    return to_json(res)
