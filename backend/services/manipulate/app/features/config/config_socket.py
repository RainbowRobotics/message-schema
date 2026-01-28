from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    to_json,
)

from .config_module import (
    ConfigService,
)
from .config_schema import (
    Request_Save_Area_ParameterPD,
    Request_Save_Collision_ParameterPD,
    Request_Save_Direct_Teach_SensitivityPD,
    Request_Save_Gravity_ParameterPD,
    Request_Save_Robot_CodePD,
    Request_Save_SelfColl_ParameterPD,
    Request_Save_Tool_List_ParameterPD,
    Request_Save_User_FramePD,
    Request_Set_Free_DrivePD,
    Request_Set_Joint_ImpedancePD,
    Request_Set_Out_Collision_ParaPD,
    Request_Set_Self_Collision_ParaPD,
    Request_Set_ShiftPD,
    Request_Set_Tool_ListPD,
    Request_Set_User_Frame_3PointsPD,
    Request_Set_User_Frame_6DofPD,
    Request_Set_User_Frame_TCPPD,
    Request_Set_User_FramePD,
)

config_socket_router = RbSocketIORouter()
config_service = ConfigService()

# ==============================================================================
# 1. General Robot Configuration (Arm, Control Box, Code)
# ==============================================================================

@config_socket_router.on("{robot_model}/call_config_robotarm")
async def on_call_config_robotarm(data, robot_model: str):
    return to_json(await config_service.call_config_robotarm(robot_model))


@config_socket_router.on("{robot_model}/call_config_controlbox")
async def on_call_config_controlbox(data, robot_model: str):
    return to_json(await config_service.call_config_controlbox(robot_model))


@config_socket_router.on("{robot_model}/save_robot_code")
async def on_save_robot_code(data: Request_Save_Robot_CodePD, robot_model: str):
    return to_json(await config_service.save_robot_code(robot_model, request=data))


# ==============================================================================
# 2. Tool & TCP Configuration
# ==============================================================================

@config_socket_router.on("{robot_model}/call_config_toollist")
async def on_call_config_toollist(data, robot_model: str):
    return to_json(await config_service.call_config_toollist(robot_model))


@config_socket_router.on("{robot_model}/set_toollist_num")
async def on_set_toollist_num(data: Request_Set_Tool_ListPD, robot_model: str):
    return to_json(await config_service.set_toollist_num(robot_model, request=data))


@config_socket_router.on("{robot_model}/save_tool_list_parameter")
async def on_save_tool_list_parameter(data: Request_Save_Tool_List_ParameterPD, robot_model: str):
    return to_json(await config_service.save_tool_list_parameter(robot_model, request=data))


# ==============================================================================
# 3. Coordinate Systems (User Frame, Area)
# ==============================================================================

@config_socket_router.on("{robot_model}/rb_api/user_frames")
async def on_rb_api_user_frames(data, robot_model: str):
    return to_json(await config_service.get_user_frames(robot_model))


@config_socket_router.on("{robot_model}/set_userframe_num")
async def on_set_userframe_num(data: Request_Set_User_FramePD, robot_model: str):
    return to_json(await config_service.set_userframe_num(robot_model, request=data))


@config_socket_router.on("{robot_model}/save_user_frame_parameter")
async def on_save_user_frame_parameter(data: Request_Save_User_FramePD, robot_model: str):
    return to_json(await config_service.save_user_frame_parameter(robot_model, request=data))


@config_socket_router.on("{robot_model}/set_userframe_6dof")
async def on_set_userframe_6dof(data: Request_Set_User_Frame_6DofPD, robot_model: str):
    return to_json(await config_service.set_userframe_6dof(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/set_userframe_tcp")
async def on_set_userframe_tcp(data: Request_Set_User_Frame_TCPPD, robot_model: str):
    return to_json(await config_service.set_userframe_tcp(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/set_userframe_3points")
async def on_set_userframe_3points(data: Request_Set_User_Frame_3PointsPD, robot_model: str):
    return to_json(await config_service.set_userframe_3points(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/save_area_parameter")
async def on_save_area_parameter(data: Request_Save_Area_ParameterPD, robot_model: str):
    return to_json(await config_service.save_area_parameter(robot_model, request=data))


# ==============================================================================
# 4. Safety & Sensitivity
# ==============================================================================

@config_socket_router.on("{robot_model}/save_direct_teach_sensitivity")
async def on_save_direct_teach_sensitivity(data: Request_Save_Direct_Teach_SensitivityPD, robot_model: str):
    return to_json(await config_service.save_direct_teach_sensitivity(robot_model, request=data))


@config_socket_router.on("{robot_model}/save_collision_parameter")
async def on_save_collision_parameter(data: Request_Save_Collision_ParameterPD, robot_model: str):
    return to_json(await config_service.save_collision_parameter(robot_model, request=data))


@config_socket_router.on("{robot_model}/save_selfcoll_parameter")
async def on_save_selfcoll_parameter(data: Request_Save_SelfColl_ParameterPD, robot_model: str):
    return to_json(await config_service.save_selfcoll_parameter(robot_model, request=data))


@config_socket_router.on("{robot_model}/set_out_collision_para")
async def on_set_out_collision_para(data: Request_Set_Out_Collision_ParaPD, robot_model: str):
    return to_json(await config_service.set_out_collision_para(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/set_self_collision_para")
async def on_set_self_collision_para(data: Request_Set_Self_Collision_ParaPD, robot_model: str):
    return to_json(await config_service.set_self_collision_para(robot_model=robot_model, request=data))


# ==============================================================================
# 5. Motion Control (Gravity, Shift, Impedance, Freedrive)
# ==============================================================================

@config_socket_router.on("{robot_model}/save_gravity_parameter")
async def on_save_gravity_parameter(data: Request_Save_Gravity_ParameterPD, robot_model: str):
    return to_json(await config_service.save_gravity_parameter(robot_model, request=data))


@config_socket_router.on("{robot_model}/set_shift")
async def on_set_shift(data: Request_Set_ShiftPD, robot_model: str):
    return to_json(await config_service.set_shift(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/set_joint_impedance")
async def on_set_joint_impedance(data: Request_Set_Joint_ImpedancePD, robot_model: str):
    return to_json(await config_service.set_joint_impedance(robot_model=robot_model, request=data))


@config_socket_router.on("{robot_model}/set_freedrive")
async def on_set_freedrive(data: Request_Set_Free_DrivePD, robot_model: str):
    return to_json(await config_service.set_freedrive(robot_model=robot_model, request=data))
