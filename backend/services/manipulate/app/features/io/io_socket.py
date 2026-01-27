from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)

from .io_module import (
    IoService,
)

io_socket_router = RbSocketIORouter()
io_service = IoService()


# ==============================================================================
# 1. Side IO Control
# ==============================================================================

@io_socket_router.on("{robot_model}/call_side_dout")
async def on_call_side_dout(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_toggle")
async def on_call_side_dout_toggle(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_toggle(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_bitcombination")
async def on_call_side_dout_bitcombination(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_bitcombination(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_pulse")
async def on_call_side_dout_pulse(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_pulse(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout")
async def on_call_multiple_side_dout(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_toggle")
async def on_call_multiple_side_dout_toggle(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_toggle(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_bitcombination")
async def on_call_multiple_side_dout_bitcombination(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_bitcombination(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_pulse")
async def on_call_multiple_side_dout_pulse(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_pulse(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_aout")
async def on_call_side_aout(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_aout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_aout")
async def on_call_multiple_side_aout(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_aout(robot_model, request=dict_data))


# ==============================================================================
# 2. IO Configuration (DIN/DOUT)
# ==============================================================================

@io_socket_router.on("{robot_model}/save_side_dout_function")
async def on_save_side_dout_function(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_dout_function(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/save_side_din_function")
async def on_save_side_din_function(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_din_function(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/save_side_din_filter")
async def on_save_side_din_filter(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_din_filter(robot_model, request=dict_data))


# ==============================================================================
# 3. Flange IO Control
# ==============================================================================

@io_socket_router.on("{robot_model}/call_flange_power")
async def on_call_flange_power(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_flange_power(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_flange_dout")
async def on_call_flange_dout(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_flange_dout(robot_model, request=dict_data))
