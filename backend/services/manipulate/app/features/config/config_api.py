from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_schemas.base import Response_ReturnValuePD

from .config_module import ConfigService
from .config_schema import (
    Request_Save_Area_ParameterPD,
    Request_Save_Collision_ParameterPD,
    Request_Save_Direct_Teach_SensitivityPD,
    Request_Save_Gravity_ParameterPD,
    Request_Save_SelfColl_ParameterPD,
    Request_Save_SideDin_FilterPD,
    Request_Save_SideDin_FunctionPD,
    Request_Save_SideDout_FunctionPD,
    Request_Save_Tool_List_ParameterPD,
    Request_Save_User_FramePD,
    Request_Set_Tool_ListPD,
    Request_Set_User_FramePD,
    Response_CallConfigControlBoxPD,
    Response_CallConfigRobotArmPD,
    Response_CallConfigToolListPD,
    Response_UserFrameParameterPD,
)

config_service = ConfigService()

config_router = APIRouter(tags=["Config"])


@config_router.get(
    "/{robot_model}/call_config_toollist", response_model=Response_CallConfigToolListPD
)
async def config_toollist(robot_model: str):
    res = config_service.config_tool_list(robot_model)
    return JSONResponse(res)


@config_router.post("/{robot_model}/set_toolist_num", response_model=Response_ReturnValuePD)
async def change_toollist(robot_model: str, *, request: Request_Set_Tool_ListPD):
    res = config_service.set_toolist_num(robot_model, request=request)
    return JSONResponse(res)


@config_router.get(
    "/{robot_model}/call_config_robotarm", response_model=Response_CallConfigRobotArmPD
)
async def config_robotarm(robot_model: str):
    res = config_service.config_robot_arm(robot_model)
    return JSONResponse(res)


@config_router.get(
    "/{robot_model}/call_config_controlbox", response_model=Response_CallConfigControlBoxPD
)
async def config_controlbox(robot_model: str):
    res = config_service.config_control_box(robot_model)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_area_parameter", response_model=Response_ReturnValuePD)
async def save_area_parameter(robot_model: str, *, request: Request_Save_Area_ParameterPD):
    res = config_service.save_area_parameter(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_tool_parameter", response_model=Response_ReturnValuePD)
async def save_tool_list_parameter(
    robot_model: str, *, request: Request_Save_Tool_List_ParameterPD
):
    res = config_service.save_tool_list_parameter(robot_model, request=request)
    return JSONResponse(res)


@config_router.post(
    "/{robot_model}/save_direct_teach_sensitivity", response_model=Response_ReturnValuePD
)
async def save_direct_teach_sensitivity(
    robot_model: str, *, request: Request_Save_Direct_Teach_SensitivityPD
):
    res = config_service.save_direct_teach_sensitivity(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_side_din_filter", response_model=Response_ReturnValuePD)
async def save_side_din_filter(robot_model: str, *, request: Request_Save_SideDin_FilterPD):
    res = config_service.save_side_din_filter(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_side_din_function", response_model=Response_ReturnValuePD)
async def save_side_din_function(robot_model: str, *, request: Request_Save_SideDin_FunctionPD):
    res = config_service.save_side_din_function(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_side_dout_function", response_model=Response_ReturnValuePD)
async def save_side_dout_function(robot_model: str, *, request: Request_Save_SideDout_FunctionPD):
    res = config_service.save_side_dout_function(robot_model, request=request)
    return JSONResponse(res)


@config_router.post(
    "/{robot_model}/save_collision_parameter", response_model=Response_ReturnValuePD
)
async def save_collision_parameter(
    robot_model: str, *, request: Request_Save_Collision_ParameterPD
):
    res = config_service.save_collision_parameter(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_selfcoll_parameter", response_model=Response_ReturnValuePD)
async def save_selfcoll_parameter(robot_model: str, *, request: Request_Save_SelfColl_ParameterPD):
    res = config_service.save_selfcoll_parameter(robot_model, request=request)
    return JSONResponse(res)


@config_router.post(
    "/{robot_model}/save_user_frame_parameter", response_model=Response_ReturnValuePD
)
async def save_user_frame_parameter(robot_model: str, *, request: Request_Save_User_FramePD):
    res = config_service.save_user_frame_parameter(robot_model, request=request)

    return JSONResponse(res)


@config_router.get(
    "/{robot_model}/rb_api/user_frames", response_model=Response_UserFrameParameterPD
)
async def get_user_frames(robot_model: str):
    res = config_service.get_user_frames(robot_model)

    return JSONResponse(res)


@config_router.post("/{robot_model}/set_userframe_num", response_model=Response_ReturnValuePD)
async def set_userframe_num(robot_model: str, *, request: Request_Set_User_FramePD):
    res = config_service.set_userframe_num(robot_model, request=request)
    return JSONResponse(res)


@config_router.post("/{robot_model}/save_gravity_parameter", response_model=Response_ReturnValuePD)
async def save_gravity_parameter(robot_model: str, *, request: Request_Save_Gravity_ParameterPD):
    res = config_service.save_gravity_parameter(robot_model, request=request)
    return JSONResponse(res)
