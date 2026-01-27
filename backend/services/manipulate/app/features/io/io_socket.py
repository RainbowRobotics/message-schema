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
from .io_schema import (
    Request_Flange_Digital_OutPD,
    Request_Flange_PowerPD,
    Request_Multiple_SideAoutPD,
    Request_Multiple_SideDoutBitcombinationPD,
    Request_Multiple_SideDoutPD,
    Request_Multiple_SideDoutPulsePD,
    Request_Multiple_SideDoutTogglePD,
    Request_Save_SideDin_FilterCountPD,
    Request_Save_SideDin_SpecialFuncPD,
    Request_Save_SideDout_SpecialFuncPD,
    Request_SideAout_GeneralPD,
    Request_SideDout_BitcombinationPD,
    Request_SideDout_GeneralPD,
    Request_SideDout_PulsePD,
    Request_SideDout_TogglePD,
)

io_socket_router = RbSocketIORouter()
io_service = IoService()


# ==============================================================================
# 1. Side IO Control
# ==============================================================================

@io_socket_router.on("{robot_model}/call_side_dout")
async def on_call_side_dout(data: Request_SideDout_GeneralPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_toggle")
async def on_call_side_dout_toggle(data: Request_SideDout_TogglePD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_toggle(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_bitcombination")
async def on_call_side_dout_bitcombination(data: Request_SideDout_BitcombinationPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_bitcombination(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_dout_pulse")
async def on_call_side_dout_pulse(data: Request_SideDout_PulsePD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_dout_pulse(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout")
async def on_call_multiple_side_dout(data: Request_Multiple_SideDoutPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_toggle")
async def on_call_multiple_side_dout_toggle(data: Request_Multiple_SideDoutTogglePD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_toggle(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_bitcombination")
async def on_call_multiple_side_dout_bitcombination(data: Request_Multiple_SideDoutBitcombinationPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_bitcombination(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_dout_pulse")
async def on_call_multiple_side_dout_pulse(data: Request_Multiple_SideDoutPulsePD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_dout_pulse(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_side_aout")
async def on_call_side_aout(data: Request_SideAout_GeneralPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_side_aout(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_multiple_side_aout")
async def on_call_multiple_side_aout(data: Request_Multiple_SideAoutPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_multiple_side_aout(robot_model, request=dict_data))


# ==============================================================================
# 2. IO Configuration (DIN/DOUT)
# ==============================================================================

@io_socket_router.on("{robot_model}/save_side_dout_function")
async def on_save_side_dout_function(data: Request_Save_SideDout_SpecialFuncPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_dout_function(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/save_side_din_function")
async def on_save_side_din_function(data: Request_Save_SideDin_SpecialFuncPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_din_function(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/save_side_din_filter")
async def on_save_side_din_filter(data: Request_Save_SideDin_FilterCountPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.save_side_din_filter(robot_model, request=dict_data))


# ==============================================================================
# 3. Flange IO Control
# ==============================================================================

@io_socket_router.on("{robot_model}/call_flange_power")
async def on_call_flange_power(data: Request_Flange_PowerPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_flange_power(robot_model, request=dict_data))


@io_socket_router.on("{robot_model}/call_flange_dout")
async def on_call_flange_dout(data: Request_Flange_Digital_OutPD, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await io_service.call_flange_dout(robot_model, request=dict_data))
