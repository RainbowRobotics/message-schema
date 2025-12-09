from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
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
    Response_CallConfigControlBoxPD,
    Response_CallConfigRobotArmPD,
    Response_CallConfigToolListPD,
    Response_UserFrameParameterPD,
)

config_service = ConfigService()
config_router = APIRouter(tags=["Config"])

# ==============================================================================
# 1. General Robot Configuration (Arm, Control Box, Code)
# ==============================================================================

@config_router.get("/{robot_model}/call_config_robotarm", response_model=Response_CallConfigRobotArmPD)
async def call_config_robotarm(robot_model: str):
    return JSONResponse(await config_service.call_config_robotarm(robot_model=robot_model))


@config_router.get("/{robot_model}/call_config_controlbox", response_model=Response_CallConfigControlBoxPD)
async def call_config_controlbox(robot_model: str):
    return JSONResponse(await config_service.call_config_controlbox(robot_model=robot_model))


@config_router.post("/{robot_model}/save_robot_code", response_model=Response_ReturnValuePD, deprecated=True)
async def save_robot_code(robot_model: str, request: Request_Save_Robot_CodePD):
    return JSONResponse(await config_service.save_robot_code(robot_model=robot_model, request=request))


# ==============================================================================
# 2. Tool & TCP Configuration
# ==============================================================================

@config_router.get("/{robot_model}/call_config_toollist", response_model=Response_CallConfigToolListPD)
async def call_config_toollist(robot_model: str):
    return JSONResponse(await config_service.call_config_toollist(robot_model=robot_model))


@config_router.post("/{robot_model}/set_toollist_num", response_model=Response_ReturnValuePD)
async def set_toollist_num(robot_model: str, request: Request_Set_Tool_ListPD):
    return JSONResponse(await config_service.set_toollist_num(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/save_tool_parameter", response_model=Response_ReturnValuePD)
async def save_tool_list_parameter(robot_model: str, request: Request_Save_Tool_List_ParameterPD):
    return JSONResponse(await config_service.save_tool_list_parameter(robot_model=robot_model, request=request))


# ==============================================================================
# 3. Coordinate Systems (User Frame, Area)
# ==============================================================================

@config_router.get("/{robot_model}/rb_api/user_frames", response_model=Response_UserFrameParameterPD)
async def get_user_frames(robot_model: str):
    return JSONResponse(await config_service.get_user_frames(robot_model=robot_model))


@config_router.post("/{robot_model}/set_userframe_num", response_model=Response_ReturnValuePD)
async def set_userframe_num(robot_model: str, request: Request_Set_User_FramePD):
    return JSONResponse(await config_service.set_userframe_num(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/save_user_frame_parameter", response_model=Response_ReturnValuePD)
async def save_user_frame_parameter(robot_model: str, request: Request_Save_User_FramePD):
    return JSONResponse(await config_service.save_user_frame_parameter(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_userframe_6dof", response_model=Response_ReturnValuePD)
async def set_userframe_6dof(robot_model: str, request: Request_Set_User_Frame_6DofPD):
    return JSONResponse(await config_service.set_userframe_6dof(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_userframe_tcp", response_model=Response_ReturnValuePD)
async def set_userframe_tcp(robot_model: str, request: Request_Set_User_Frame_TCPPD):
    return JSONResponse(await config_service.set_userframe_tcp(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_userframe_3points", response_model=Response_ReturnValuePD)
async def set_userframe_3points(robot_model: str, request: Request_Set_User_Frame_3PointsPD):
    return JSONResponse(await config_service.set_userframe_3points(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/save_area_parameter", response_model=Response_ReturnValuePD)
async def save_area_parameter(robot_model: str, request: Request_Save_Area_ParameterPD):
    return JSONResponse(await config_service.save_area_parameter(robot_model=robot_model, request=request))


# ==============================================================================
# 4. Safety & Sensitivity
# ==============================================================================

@config_router.post("/{robot_model}/save_direct_teach_sensitivity", response_model=Response_ReturnValuePD)
async def save_direct_teach_sensitivity(robot_model: str, request: Request_Save_Direct_Teach_SensitivityPD):
    return JSONResponse(await config_service.save_direct_teach_sensitivity(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/save_collision_parameter", response_model=Response_ReturnValuePD)
async def save_collision_parameter(robot_model: str, request: Request_Save_Collision_ParameterPD):
    return JSONResponse(await config_service.save_collision_parameter(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/save_selfcoll_parameter", response_model=Response_ReturnValuePD)
async def save_selfcoll_parameter(robot_model: str, request: Request_Save_SelfColl_ParameterPD):
    return JSONResponse(await config_service.save_selfcoll_parameter(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_out_collision_para", response_model=Response_ReturnValuePD)
async def set_out_collision_parameter(robot_model: str, request: Request_Set_Out_Collision_ParaPD):
    return JSONResponse(await config_service.set_out_collision_para(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_self_collision_para", response_model=Response_ReturnValuePD)
async def set_self_collision_parameter(robot_model: str, request: Request_Set_Self_Collision_ParaPD):
    return JSONResponse(await config_service.set_self_collision_para(robot_model=robot_model, request=request))


# ==============================================================================
# 5. Motion Control (Gravity, Shift, Impedance, Freedrive)
# ==============================================================================

@config_router.post("/{robot_model}/save_gravity_parameter", response_model=Response_ReturnValuePD)
async def save_gravity_parameter(robot_model: str, request: Request_Save_Gravity_ParameterPD):
    return JSONResponse(await config_service.save_gravity_parameter(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_shift", response_model=Response_ReturnValuePD)
async def set_shift(robot_model: str, request: Request_Set_ShiftPD):
    return JSONResponse(await config_service.set_shift(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_joint_impedance", response_model=Response_ReturnValuePD)
async def set_joint_impedance(robot_model: str, request: Request_Set_Joint_ImpedancePD):
    return JSONResponse(await config_service.set_joint_impedance(robot_model=robot_model, request=request))


@config_router.post("/{robot_model}/set_freedrive", response_model=Response_ReturnValuePD)
async def set_freedrive(robot_model: str, request: Request_Set_Free_DrivePD):
    return JSONResponse(await config_service.set_freedrive(robot_model=robot_model, request=request))
