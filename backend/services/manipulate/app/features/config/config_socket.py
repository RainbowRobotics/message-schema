from rb_socketio import RbSocketIORouter
from rb_utils.parser import t_to_dict, to_json

from .config_module import ConfigService

config_socket_router = RbSocketIORouter()
config_service = ConfigService()


@config_socket_router.on("{robot_model}/call_config_toollist")
def on_call_config_toollist(data, robot_model: str):
    res = config_service.config_tool_list(robot_model)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_toolist_num")
def on_set_toolist_num(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.set_toolist_num(robot_model, request=dict_data)
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
    dict_data = t_to_dict(data)

    res = config_service.save_area_parameter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_tool_list_parameter")
def on_save_tool_list_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_tool_list_parameter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_direct_teach_sensitivity")
def on_save_direct_teach_sensitivity(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_direct_teach_sensitivity(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_din_filter")
def on_save_side_din_filter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_side_din_filter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_din_function")
def on_save_side_din_function(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_side_din_function(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_side_dout_function")
def on_save_side_dout_function(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_side_dout_function(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_collision_parameter")
def on_save_collision_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_collision_parameter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_selfcoll_parameter")
def on_save_selfcoll_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_selfcoll_parameter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_user_frame_parameter")
def on_save_user_frame_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_user_frame_parameter(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/rb_api/user_frames")
def on_rb_api_user_frames(data, robot_model: str):
    res = config_service.get_user_frames(robot_model)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_userframe_num")
def on_set_userframe_num(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.set_userframe_num(robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/save_gravity_parameter")
def on_save_gravity_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)

    res = config_service.save_gravity_parameter(robot_model, request=dict_data)
    return to_json(res)



@config_socket_router.on("{robot_model}/set_shift")
async def on_set_shift(data, robot_model: str):
    dict_data = t_to_dict(data)
    
    res = await config_service.set_shift(robot_model=robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_out_collision_parameter")
async def on_set_out_collision_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)
    
    res = await config_service.set_shift(robot_model=robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_self_collision_parameter")
async def on_set_self_collision_parameter(data, robot_model: str):
    dict_data = t_to_dict(data)
    
    res = await config_service.set_shift(robot_model=robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_joint_impedance")
async def on_set_joint_impedance(data, robot_model: str):
    dict_data = t_to_dict(data)
    
    res = await config_service.set_shift(robot_model=robot_model, request=dict_data)
    return to_json(res)


@config_socket_router.on("{robot_model}/set_free_drive")
async def on_set_free_drive(data, robot_model: str):
    dict_data = t_to_dict(data)
    
    res = await config_service.set_shift(robot_model=robot_model, request=dict_data)
    return to_json(res)