from rb_socketio import RbSocketIORouter
from utils.parser import to_json

from .config_module import ConfigService

config_socket_router = RbSocketIORouter()
config_service = ConfigService()


@config_socket_router.on("{robot_model}/call_config_toollist")
def on_call_config_toollist(data, robot_model: str):
    res = config_service.config_tool_list(robot_model)
    return to_json(res)


@config_socket_router.on("{robot_model}/call_config_robotarm")
def on_call_config_robotarm(data, robot_model: str):
    res = config_service.config_robot_arm(robot_model)
    return to_json(res)


@config_socket_router.on("{robot_model}/call_config_controlbox")
def on_call_config_controlbox(data, robot_model: str):
    res = config_service.config_control_box(robot_model)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_area_parameter")
def on_save_area_parameter(data, robot_model: str):
    res = config_service.save_area_parameter(robot_model, request=data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_tool_list_parameter")
def on_save_tool_list_parameter(data, robot_model: str):
    res = config_service.save_tool_list_parameter(robot_model, request=data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_direct_teach_sensitivity")
def on_save_direct_teach_sensitivity(data, robot_model: str):
    res = config_service.save_direct_teach_sensitivity(robot_model, request=data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_din_filter")
def on_save_side_din_filter(data, robot_model: str):
    res = config_service.save_side_din_filter(robot_model, request=data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_din_function")
def on_save_side_din_function(data, robot_model: str):
    res = config_service.save_side_din_function(robot_model, request=data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_dout_function")
def on_save_side_dout_function(data, robot_model: str):
    res = config_service.save_side_dout_function(robot_model, request=data)
    return to_json(res)
