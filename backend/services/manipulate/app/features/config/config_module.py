from flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from flat_buffers.IPC.Request_CallConfigControlBox import Request_CallConfigControlBoxT
from flat_buffers.IPC.Request_CallConfigRobotArm import Request_CallConfigRobotArmT
from flat_buffers.IPC.Request_CallConfigToolList import Request_CallConfigToolListT
from flat_buffers.IPC.Request_Save_Area_Para import Request_Save_Area_ParaT
from flat_buffers.IPC.Request_Save_Collision_Parameter import Request_Save_Collision_ParameterT
from flat_buffers.IPC.Request_Save_Direct_Teach_Sensitivity import (
    Request_Save_Direct_Teach_SensitivityT,
)
from flat_buffers.IPC.Request_Save_SelfColl_Parameter import Request_Save_SelfColl_ParameterT
from flat_buffers.IPC.Request_Save_SideDin_FilterCount import Request_Save_SideDin_FilterCountT
from flat_buffers.IPC.Request_Save_SideDin_SpecialFunc import Request_Save_SideDin_SpecialFuncT
from flat_buffers.IPC.Request_Save_SideDout_SpecialFunc import Request_Save_SideDout_SpecialFuncT
from flat_buffers.IPC.Request_Save_Tool_List_Para import Request_Save_Tool_List_ParaT
from flat_buffers.IPC.Response_CallConfigControlBox import Response_CallConfigControlBoxT
from flat_buffers.IPC.Response_CallConfigRobotArm import Response_CallConfigRobotArmT
from flat_buffers.IPC.Response_CallConfigToolList import Response_CallConfigToolListT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh.client import ZenohClient

from .config_schema import (
    Request_Save_Area_ParameterPD,
    Request_Save_Collision_ParameterPD,
    Request_Save_Direct_Teach_SensitivityPD,
    Request_Save_SelfColl_ParameterPD,
    Request_Save_SideDin_FilterPD,
    Request_Save_SideDin_FunctionPD,
    Request_Save_SideDout_FunctionPD,
    Request_Save_Tool_List_ParameterPD,
)

zenoh_client = ZenohClient()


class ConfigService:
    def __init__(self):
        pass

    def config_tool_list(self, robot_model: str):
        req = Request_CallConfigToolListT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_toollist",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigToolListT,
            flatbuffer_buf_size=8,
        )

        return res

    def config_robot_arm(self, robot_model: str):
        req = Request_CallConfigRobotArmT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_robotarm",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigRobotArmT,
            flatbuffer_buf_size=8,
        )

        return res

    def config_control_box(self, robot_model: str):
        req = Request_CallConfigControlBoxT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_controlbox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigControlBoxT,
            flatbuffer_buf_size=8,
        )

        return res

    def save_area_parameter(self, robot_model: str, *, request: Request_Save_Area_ParameterPD):
        req = Request_Save_Area_ParaT()
        req.areaNo = request.area_no
        req.areaName = request.area_name
        req.areaType = request.area_type
        req.areaX = request.area_x
        req.areaY = request.area_y
        req.areaZ = request.area_z
        req.areaRX = request.area_rx
        req.areaRY = request.area_ry
        req.areaRZ = request.area_rz
        req.areaPara0 = request.area_para_0
        req.areaPara1 = request.area_para_1
        req.areaPara2 = request.area_para_2

        res = zenoh_client.query_one(
            f"{robot_model}/save_area_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=100,
        )

        return res

    def save_tool_list_parameter(
        self, robot_model: str, *, request: Request_Save_Tool_List_ParameterPD
    ):
        req = Request_Save_Tool_List_ParaT()
        req.toolNo = request.tool_no
        req.toolName = request.tool_name
        req.tcpX = request.tcp_x
        req.tcpY = request.tcp_y
        req.tcpZ = request.tcp_z
        req.tcpRx = request.tcp_rx
        req.tcpRy = request.tcp_ry
        req.tcpRz = request.tcp_rz
        req.massM = request.mass_m
        req.massX = request.mass_x
        req.massY = request.mass_y
        req.massZ = request.mass_z
        req.boxType = request.box_type
        req.boxPara0 = request.box_para_0
        req.boxPara1 = request.box_para_1
        req.boxPara2 = request.box_para_2
        req.boxPara3 = request.box_para_3
        req.boxPara4 = request.box_para_4
        req.boxPara5 = request.box_para_5
        req.boxPara6 = request.box_para_6
        req.boxPara7 = request.box_para_7
        req.boxPara8 = request.box_para_8

        res = zenoh_client.query_one(
            f"{robot_model}/save_tool_list_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=160,
        )

        return res

    def save_direct_teach_sensitivity(
        self, robot_model: str, *, request: Request_Save_Direct_Teach_SensitivityPD
    ):
        req = Request_Save_Direct_Teach_SensitivityT()
        nj = N_JOINT_fT()
        nj.f = request.sensitivity
        req.sensitivity = nj

        res = zenoh_client.query_one(
            f"{robot_model}/save_direct_teach_sensitivity",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res

    def save_side_din_filter(self, robot_model: str, *, request: Request_Save_SideDin_FilterPD):
        req = Request_Save_SideDin_FilterCountT()
        req.portNum = request.port_num
        req.desiredCount = request.desired_count

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_filter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res

    def save_side_din_function(self, robot_model: str, *, request: Request_Save_SideDin_FunctionPD):
        req = Request_Save_SideDin_SpecialFuncT()
        req.portNum = request.port_num
        req.desiredFunction = request.desired_function

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res

    def save_side_dout_function(
        self, robot_model: str, *, request: Request_Save_SideDout_FunctionPD
    ):
        req = Request_Save_SideDout_SpecialFuncT()
        req.portNum = request.port_num
        req.desiredFunction = request.desired_function

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_dout_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res

    def save_collision_parameter(
        self, robot_model: str, *, request: Request_Save_Collision_ParameterPD
    ):
        req = Request_Save_Collision_ParameterT()
        req.onoff = request.onoff
        req.react = request.react
        req.threshold = request.threshold

        res = zenoh_client.query_one(
            f"{robot_model}/save_collision_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res

    def save_selfcoll_parameter(
        self, robot_model: str, *, request: Request_Save_SelfColl_ParameterPD
    ):
        req = Request_Save_SelfColl_ParameterT()
        req.mode = request.mode
        req.dist_internal = request.dist_internal
        req.dist_external = request.dist_external

        res = zenoh_client.query_one(
            f"{robot_model}/save_selfcoll_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res
