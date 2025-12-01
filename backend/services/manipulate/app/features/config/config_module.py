from app.socket.socket_client import socket_client
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from rb_flat_buffers.IPC.Request_CallConfigControlBox import Request_CallConfigControlBoxT
from rb_flat_buffers.IPC.Request_CallConfigRobotArm import Request_CallConfigRobotArmT
from rb_flat_buffers.IPC.Request_CallConfigToolList import Request_CallConfigToolListT
from rb_flat_buffers.IPC.Request_Save_Area_Para import Request_Save_Area_ParaT
from rb_flat_buffers.IPC.Request_Save_Collision_Parameter import Request_Save_Collision_ParameterT
from rb_flat_buffers.IPC.Request_Save_Direct_Teach_Sensitivity import (
    Request_Save_Direct_Teach_SensitivityT,
)
from rb_flat_buffers.IPC.Request_Save_Gravity_Parameter import Request_Save_Gravity_ParameterT
from rb_flat_buffers.IPC.Request_Save_Robot_Code import Request_Save_Robot_CodeT
from rb_flat_buffers.IPC.Request_Save_SelfColl_Parameter import Request_Save_SelfColl_ParameterT
from rb_flat_buffers.IPC.Request_Save_SideDin_FilterCount import Request_Save_SideDin_FilterCountT
from rb_flat_buffers.IPC.Request_Save_SideDin_SpecialFunc import Request_Save_SideDin_SpecialFuncT
from rb_flat_buffers.IPC.Request_Save_SideDout_SpecialFunc import Request_Save_SideDout_SpecialFuncT
from rb_flat_buffers.IPC.Request_Save_Tool_List_Para import Request_Save_Tool_List_ParaT
from rb_flat_buffers.IPC.Request_Save_User_Frame import Request_Save_User_FrameT
from rb_flat_buffers.IPC.Request_Set_Free_Drive import Request_Set_Free_DriveT
from rb_flat_buffers.IPC.Request_Set_Joint_Impedance import Request_Set_Joint_ImpedanceT
from rb_flat_buffers.IPC.Request_Set_Out_Collision_Para import Request_Set_Out_Collision_ParaT
from rb_flat_buffers.IPC.Request_Set_Self_Collision_Para import Request_Set_Self_Collision_ParaT
from rb_flat_buffers.IPC.Request_Set_Shift import Request_Set_ShiftT
from rb_flat_buffers.IPC.Request_Set_Tool_List import Request_Set_Tool_ListT
from rb_flat_buffers.IPC.Request_Set_User_Frame import Request_Set_User_FrameT
from rb_flat_buffers.IPC.Request_Set_User_Frame_3Points import Request_Set_User_Frame_3PointsT
from rb_flat_buffers.IPC.Request_Set_User_Frame_6Dof import Request_Set_User_Frame_6DofT
from rb_flat_buffers.IPC.Request_Set_User_Frame_TCP import Request_Set_User_Frame_TCPT
from rb_flat_buffers.IPC.Response_CallConfigControlBox import Response_CallConfigControlBoxT
from rb_flat_buffers.IPC.Response_CallConfigRobotArm import Response_CallConfigRobotArmT
from rb_flat_buffers.IPC.Response_CallConfigToolList import Response_CallConfigToolListT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_utils.asyncio_helper import fire_and_log
from rb_utils.parser import t_to_dict, to_json
from rb_zenoh.client import ZenohClient

from .config_schema import (
    Request_Save_Area_ParameterPD,
    Request_Save_Collision_ParameterPD,
    Request_Save_Direct_Teach_SensitivityPD,
    Request_Save_Gravity_ParameterPD,
    Request_Save_Robot_CodePD,
    Request_Save_SelfColl_ParameterPD,
    Request_Save_SideDin_FilterPD,
    Request_Save_SideDin_FunctionPD,
    Request_Save_SideDout_FunctionPD,
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
)

zenoh_client = ZenohClient()


class ConfigService(BaseService):
    def __init__(self):
        pass

    async def config_tool_list(self, robot_model: str):
        req = Request_CallConfigToolListT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_toollist",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigToolListT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]



    def socket_emit_config_toollist(self, robot_model: str):
        config_toollist_res = self.config_tool_list(robot_model)

        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_toollist", to_json(config_toollist_res))
        )


    async def config_robot_arm(self, robot_model: str):
        req = Request_CallConfigRobotArmT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_robotarm",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigRobotArmT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    def socket_emit_config_robot_arm(self, robot_model: str):
        config_robot_arm_res = self.config_robot_arm(robot_model)

        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_robotarm", to_json(config_robot_arm_res))
        )


    async def config_control_box(self, robot_model: str):
        req = Request_CallConfigControlBoxT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_config_controlbox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigControlBoxT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    async def socket_emit_config_control_box(self, robot_model: str, *, emit_user_frames: bool = False):
        config_control_box_res = await self.config_control_box(robot_model)

        fire_and_log(
            socket_client.emit(
                f"{robot_model}/call_config_controlbox", to_json(config_control_box_res)
            )
        )

        if emit_user_frames:
            user_frames_res = self.parse_get_user_frames(config_control_box_res)

            fire_and_log(
                socket_client.emit(f"{robot_model}/rb_api/user_frames", to_json(user_frames_res))
            )


    async def set_toollist_num(self, robot_model: str, *, request: Request_Set_Tool_ListPD):
        request_dict = {**request.model_dump()}

        req = Request_Set_Tool_ListT()
        req.targetToolNum = request_dict["target_tool_num"]

        res = zenoh_client.query_one(
            f"{robot_model}/set_toollist_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]
    
    def parse_get_user_frames(self, config_control_box_res: Response_CallConfigControlBoxPD):
        config_control_box_res_dict = t_to_dict(config_control_box_res)
        return {
            "user_frames": [
                config_control_box_res_dict["userFrame0"],
                config_control_box_res_dict["userFrame1"],
                config_control_box_res_dict["userFrame2"],
                config_control_box_res_dict["userFrame3"],
                config_control_box_res_dict["userFrame4"],
                config_control_box_res_dict["userFrame5"],
                config_control_box_res_dict["userFrame6"],
                config_control_box_res_dict["userFrame7"],
            ]
        }



    async def save_robot_code(self, robot_model: str, request: Request_Save_Robot_CodePD):
        req = Request_Save_Robot_CodeT()
        
        req.code = request.code
        req.option = request.option

        res = zenoh_client.query_one(
            f"{robot_model}/save_robot_code",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    async def save_area_parameter(self, robot_model: str, *, request: Request_Save_Area_ParameterPD):
        request_dict = {**request.model_dump()}

        req = Request_Save_Area_ParaT()
        req.areaNo = request_dict["area_no"]
        req.areaName = request_dict["area_name"]
        req.areaType = request_dict["area_type"]
        req.areaX = request_dict["area_x"]
        req.areaY = request_dict["area_y"]
        req.areaZ = request_dict["area_z"]
        req.areaRx = request_dict["area_rx"]
        req.areaRy = request_dict["area_ry"]
        req.areaRz = request_dict["area_rz"]
        req.areaPara0 = request_dict["area_para_0"]
        req.areaPara1 = request_dict["area_para_1"]
        req.areaPara2 = request_dict["area_para_2"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_area_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=100,
        )

        await self.socket_emit_config_control_box(robot_model)

        return res["dict_payload"]

    async def save_tool_list_parameter(
        self, robot_model: str, *, request: Request_Save_Tool_List_ParameterPD
    ):
        request_dict = {**request.model_dump()}

        req = Request_Save_Tool_List_ParaT()
        req.toolNo = request_dict["tool_no"]
        req.toolName = request_dict["tool_name"]
        req.tcpX = request_dict["tcp_x"]
        req.tcpY = request_dict["tcp_y"]
        req.tcpZ = request_dict["tcp_z"]
        req.tcpRx = request_dict["tcp_rx"]
        req.tcpRy = request_dict["tcp_ry"]
        req.tcpRz = request_dict["tcp_rz"]
        req.massM = request_dict["mass_m"]
        req.massX = request_dict["mass_x"]
        req.massY = request_dict["mass_y"]
        req.massZ = request_dict["mass_z"]
        req.boxType = request_dict["box_type"]
        req.boxPara0 = request_dict["box_para_0"]
        req.boxPara1 = request_dict["box_para_1"]
        req.boxPara2 = request_dict["box_para_2"]
        req.boxPara3 = request_dict["box_para_3"]
        req.boxPara4 = request_dict["box_para_4"]
        req.boxPara5 = request_dict["box_para_5"]
        req.boxPara6 = request_dict["box_para_6"]
        req.boxPara7 = request_dict["box_para_7"]
        req.boxPara8 = request_dict["box_para_8"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_tool_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=160,
        )

        self.socket_emit_config_toollist(robot_model)

        return res["dict_payload"]

    async def save_direct_teach_sensitivity(
        self, robot_model: str, *, request: Request_Save_Direct_Teach_SensitivityPD
    ):
        request_dict = t_to_dict(request)

        req = Request_Save_Direct_Teach_SensitivityT()
        nj = N_JOINT_fT()
        nj.f = request_dict["sensitivity"]
        req.sensitivity = nj

        res = zenoh_client.query_one(
            f"{robot_model}/save_direct_teach_sensitivity",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def save_side_din_filter(self, robot_model: str, *, request: Request_Save_SideDin_FilterPD):
        request_dict = t_to_dict(request)

        req = Request_Save_SideDin_FilterCountT()
        req.portNum = request_dict["port_num"]
        req.desiredCount = request_dict["desired_count"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_filter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        await self.socket_emit_config_control_box(robot_model)

        return res["dict_payload"]

    async def save_side_din_function(self, robot_model: str, *, request: Request_Save_SideDin_FunctionPD):
        request_dict = t_to_dict(request)

        req = Request_Save_SideDin_SpecialFuncT()
        req.portNum = request_dict["port_num"]
        req.desiredFunction = request_dict["desired_function"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        await self.socket_emit_config_control_box(robot_model)

        return res["dict_payload"]

    async def save_side_dout_function(
        self, robot_model: str, *, request: Request_Save_SideDout_FunctionPD
    ):
        request_dict = t_to_dict(request)

        req = Request_Save_SideDout_SpecialFuncT()
        req.portNum = request_dict["port_num"]
        req.desiredFunction = request_dict["desired_function"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_dout_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        await self.socket_emit_config_control_box(robot_model)

        return res["dict_payload"]

    async def save_collision_parameter(
        self, robot_model: str, *, request: Request_Save_Collision_ParameterPD
    ):
        request_dict = t_to_dict(request)

        req = Request_Save_Collision_ParameterT()
        req.onoff = request_dict["onoff"]
        req.react = request_dict["react"]
        req.threshold = request_dict["threshold"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_collision_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    async def save_selfcoll_parameter(
        self, robot_model: str, *, request: Request_Save_SelfColl_ParameterPD
    ):
        request_dict = t_to_dict(request)

        req = Request_Save_SelfColl_ParameterT()
        req.mode = request_dict["mode"]
        req.distInternal = request_dict["dist_internal"]
        req.distExternal = request_dict["dist_external"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_selfcoll_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    async def get_user_frames(self, robot_model: str):
        res = await self.config_control_box(robot_model)

        user_frames_res = self.parse_get_user_frames(res)

        return user_frames_res

    async def save_user_frame_parameter(self, robot_model: str, *, request: Request_Save_User_FramePD):
        request_dict = t_to_dict(request)

        req = Request_Save_User_FrameT()
        req.userfNo = request_dict["userf_no"]
        req.userfName = request_dict["userf_name"]
        req.userfX = request_dict["userf_x"]
        req.userfY = request_dict["userf_y"]
        req.userfZ = request_dict["userf_z"]
        req.userfRx = request_dict["userf_rx"]
        req.userfRy = request_dict["userf_ry"]
        req.userfRz = request_dict["userf_rz"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_user_frame_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        await self.socket_emit_config_control_box(robot_model, emit_user_frames=True)

        return res["dict_payload"]

    async def set_userframe_num(self, robot_model: str, *, request: Request_Set_User_FramePD):
        request_dict = t_to_dict(request)

        req = Request_Set_User_FrameT()
        req.userFrameNum = request_dict["user_frame_num"]

        res = zenoh_client.query_one(
            f"{robot_model}/set_userframe_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        await self.socket_emit_config_control_box(robot_model, emit_user_frames=True)

        return res["dict_payload"]

    async def save_gravity_parameter(
        self, robot_model: str, *, request: Request_Save_Gravity_ParameterPD
    ):
        request_dict = t_to_dict(request)

        req = Request_Save_Gravity_ParameterT()
        req.mode = request_dict["mode"]
        req.gx = request_dict["gx"]
        req.gy = request_dict["gy"]
        req.gz = request_dict["gz"]

        res = zenoh_client.query_one(
            f"{robot_model}/save_gravity_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    def call_reset_outcoll(self, robot_model: str):
        pass


    async def set_shift(self, *, robot_model: str, request: Request_Set_ShiftPD):
        target = t_to_dict(request.target)

        req = Request_Set_ShiftT()

        req.shiftNo = request.shift_no
        req.shiftMode = request.shift_mode
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        res = zenoh_client.query_one(
            f"{robot_model}/set_shift",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]
    
    async def set_out_collision_parameter(self, *, robot_model: str, request: Request_Set_Out_Collision_ParaPD):
        req = Request_Set_Out_Collision_ParaT()

        req.onoff = request.onoff
        req.reactMode = request.react_mode
        req.threshold = request.threshold

        res = zenoh_client.query_one(
            f"{robot_model}/set_out_collision_para",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def set_self_collision_parameter(self, *, robot_model: str, request: Request_Set_Self_Collision_ParaPD):
        req = Request_Set_Self_Collision_ParaT()

        req.mode = request.mode
        req.distInt = request.dist_int
        req.distExt = request.dist_ext

        res = zenoh_client.query_one(
            f"{robot_model}/set_self_collision_para",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def set_joint_impedance(self, *, robot_model: str, request: Request_Set_Joint_ImpedancePD):
        req = Request_Set_Joint_ImpedanceT()

        req.onoff = request.onoff

        req.stiffness = N_JOINT_fT()
        req.stiffness.f = request.stiffness.f
        
        req.torquelimit = N_JOINT_fT()
        req.torquelimit.f = request.torquelimit.f

        res = zenoh_client.query_one(
            f"{robot_model}/set_joint_impedance",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def set_freedrive(self, *, robot_model: str, request: Request_Set_Free_DrivePD):
        req = Request_Set_Free_DriveT()

        req.onoff = request.onoff
        req.sensitivity = request.sensitivity

        res = zenoh_client.query_one(
            f"{robot_model}/set_freedrive",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def set_userframe_6dof(self, *, robot_model: str, request: Request_Set_User_Frame_6DofPD):
        req = Request_Set_User_Frame_6DofT()

        req.userFrameNum= request.user_frame_num
        req.settingOption = request.setting_option
        req.targetX = request.target_x
        req.targetY = request.target_y
        req.targetZ = request.target_z
        req.targetRx = request.target_rx
        req.targetRy = request.target_ry
        req.targetRz = request.target_rz

        res = zenoh_client.query_one(
            f"{robot_model}/set_userframe_6dof",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def set_userframe_tcp(self, *, robot_model: str, request: Request_Set_User_Frame_TCPPD):
        req = Request_Set_User_Frame_TCPT()

        req.userFrameNum = request.user_frame_num
        req.settingOption = request.setting_option

        res = zenoh_client.query_one(
            f"{robot_model}/set_userframe_tcp",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def set_userframe_3points(self, *, robot_model: str, request: Request_Set_User_Frame_3PointsPD):
        req = Request_Set_User_Frame_3PointsT()

        req.userFrameNum = request.user_frame_num
        req.settingOption = request.setting_option
        req.orderOption = request.order_option
        req.point1X = request.point_1_x
        req.point1Y = request.point_1_y
        req.point1Z = request.point_1_z
        req.point2X = request.point_2_x
        req.point2Y = request.point_2_y
        req.point2Z = request.point_2_z
        req.point3X = request.point_3_x
        req.point3Y = request.point_3_y
        req.point3Z = request.point_3_z

        res = zenoh_client.query_one(
            f"{robot_model}/set_userframe_3points",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]
