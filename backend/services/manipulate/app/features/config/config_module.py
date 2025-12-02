import asyncio

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

    @staticmethod
    async def _execute_zenoh_query(topic, req_obj, res_class, buf_size):
        return await asyncio.to_thread(
            zenoh_client.query_one,
            topic,
            flatbuffer_req_obj=req_obj,
            flatbuffer_res_T_class=res_class,
            flatbuffer_buf_size=buf_size,
        )

    # ==========================================================================
    # 1. General Configuration (Robot Arm, Control Box, Code)
    # ==========================================================================

    async def call_config_robotarm(self, robot_model: str):
        req = Request_CallConfigRobotArmT()
        
        res = await self._execute_zenoh_query(
            f"{robot_model}/call_config_robotarm", req, Response_CallConfigRobotArmT, 1024
        )
        return res["dict_payload"]


    async def call_config_controlbox(self, robot_model: str):
        req = Request_CallConfigControlBoxT()
        
        res = await self._execute_zenoh_query(
            f"{robot_model}/call_config_controlbox", req, Response_CallConfigControlBoxT, 2048
        )
        return res["dict_payload"]


    async def save_robot_code(self, robot_model: str, request: Request_Save_Robot_CodePD):
        req = Request_Save_Robot_CodeT()
        req.code = request.code
        req.option = request.option

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_robot_code", req, Response_FunctionsT, 32
        )
        return res["dict_payload"]


    async def socket_emit_config_robot_arm(self, robot_model: str):
        config_robot_arm_res = await self.call_config_robotarm(robot_model)
        
        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_robotarm", to_json(config_robot_arm_res))
        )


    async def socket_emit_config_control_box(self, robot_model: str, *, emit_user_frames: bool = False):
        config_control_box_res = await self.call_config_controlbox(robot_model)

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

    # ==========================================================================
    # 2. Tool Configuration
    # ==========================================================================

    async def call_config_toollist(self, robot_model: str):
        req = Request_CallConfigToolListT()
        
        res = await self._execute_zenoh_query(
            f"{robot_model}/call_config_toollist", req, Response_CallConfigToolListT, 8
        )
        return res["dict_payload"]


    async def set_toollist_num(self, robot_model: str, *, request: Request_Set_Tool_ListPD):
        req = Request_Set_Tool_ListT()
        req.targetToolNum = request.target_tool_num

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_toollist_num", req, Response_FunctionsT, 8
        )
        return res["dict_payload"]


    async def save_tool_list_parameter(
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

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_tool_parameter", req, Response_FunctionsT, 160
        )
        await self.socket_emit_config_toollist(robot_model)
        return res["dict_payload"]


    async def socket_emit_config_toollist(self, robot_model: str):
        config_toollist_res = await self.call_config_toollist(robot_model)
        
        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_toollist", to_json(config_toollist_res))
        )

    # ==========================================================================
    # 3. Coordinate & Area Configuration
    # ==========================================================================

    async def get_user_frames(self, robot_model: str):
        res = await self.call_config_controlbox(robot_model)
        
        user_frames_res = self.parse_get_user_frames(res)
        return user_frames_res


    def parse_get_user_frames(self, config_control_box_res: Response_CallConfigControlBoxPD):
        res_data = t_to_dict(config_control_box_res)
        return {
            "user_frames": [
                res_data.get(f"userFrame{i}") for i in range(8)
            ]
        }


    async def set_userframe_num(self, robot_model: str, *, request: Request_Set_User_FramePD):
        req = Request_Set_User_FrameT()
        req.userFrameNum = request.user_frame_num

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_userframe_num", req, Response_FunctionsT, 32
        )
        await self.socket_emit_config_control_box(robot_model, emit_user_frames=True)
        return res["dict_payload"]


    async def save_user_frame_parameter(self, robot_model: str, *, request: Request_Save_User_FramePD):
        req = Request_Save_User_FrameT()
        req.userfNo = request.userf_no
        req.userfName = request.userf_name
        req.userfX = request.userf_x
        req.userfY = request.userf_y
        req.userfZ = request.userf_z
        req.userfRx = request.userf_rx
        req.userfRy = request.userf_ry
        req.userfRz = request.userf_rz

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_user_frame_parameter", req, Response_FunctionsT, 32
        )
        await self.socket_emit_config_control_box(robot_model, emit_user_frames=True)
        return res["dict_payload"]


    async def set_userframe_6dof(self, *, robot_model: str, request: Request_Set_User_Frame_6DofPD):
        req = Request_Set_User_Frame_6DofT()
        req.userFrameNum = request.user_frame_num
        req.settingOption = request.setting_option
        req.targetX = request.target_x
        req.targetY = request.target_y
        req.targetZ = request.target_z
        req.targetRx = request.target_rx
        req.targetRy = request.target_ry
        req.targetRz = request.target_rz

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_userframe_6dof", req, Response_FunctionsT, 32
        )
        return res["dict_payload"]


    async def set_userframe_tcp(self, *, robot_model: str, request: Request_Set_User_Frame_TCPPD):
        req = Request_Set_User_Frame_TCPT()
        req.userFrameNum = request.user_frame_num
        req.settingOption = request.setting_option

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_userframe_tcp", req, Response_FunctionsT, 256
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

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_userframe_3points", req, Response_FunctionsT, 48
        )
        return res["dict_payload"]


    async def save_area_parameter(self, robot_model: str, *, request: Request_Save_Area_ParameterPD):
        req = Request_Save_Area_ParaT()
        req.areaNo = request.area_no
        req.areaName = request.area_name
        req.areaType = request.area_type
        req.areaX = request.area_x
        req.areaY = request.area_y
        req.areaZ = request.area_z
        req.areaRx = request.area_rx
        req.areaRy = request.area_ry
        req.areaRz = request.area_rz
        req.areaPara0 = request.area_para_0
        req.areaPara1 = request.area_para_1
        req.areaPara2 = request.area_para_2

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_area_parameter", req, Response_FunctionsT, 100
        )
        await self.socket_emit_config_control_box(robot_model)
        return res["dict_payload"]



    # ==========================================================================
    # 4. Safety & Sensitivity
    # ==========================================================================

    async def save_direct_teach_sensitivity(
        self, robot_model: str, *, request: Request_Save_Direct_Teach_SensitivityPD
    ):
        req = Request_Save_Direct_Teach_SensitivityT()
        nj = N_JOINT_fT()
        nj.f = request.sensitivity
        req.sensitivity = nj

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_direct_teach_sensitivity", req, Response_FunctionsT, 32
        )
        return res["dict_payload"]


    async def save_collision_parameter(
        self, robot_model: str, *, request: Request_Save_Collision_ParameterPD
    ):
        req = Request_Save_Collision_ParameterT()
        req.onoff = request.onoff
        req.react = request.react
        req.threshold = request.threshold

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_collision_parameter", req, Response_FunctionsT, 12
        )
        return res["dict_payload"]


    async def save_selfcoll_parameter(
        self, robot_model: str, *, request: Request_Save_SelfColl_ParameterPD
    ):
        req = Request_Save_SelfColl_ParameterT()
        req.mode = request.mode
        req.distInternal = request.dist_internal
        req.distExternal = request.dist_external

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_selfcoll_parameter", req, Response_FunctionsT, 12
        )
        return res["dict_payload"]


    async def set_out_collision_para(self, *, robot_model: str, request: Request_Set_Out_Collision_ParaPD):
        req = Request_Set_Out_Collision_ParaT()
        req.onoff = request.onoff
        req.reactMode = request.react_mode
        req.threshold = request.threshold

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_out_collision_para", req, Response_FunctionsT, 12
        )
        return res["dict_payload"]


    async def set_self_collision_para(self, *, robot_model: str, request: Request_Set_Self_Collision_ParaPD):
        req = Request_Set_Self_Collision_ParaT()
        req.mode = request.mode
        req.distInt = request.dist_int
        req.distExt = request.dist_ext

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_self_collision_para", req, Response_FunctionsT, 12
        )
        return res["dict_payload"]

    # ==========================================================================
    # 5. Motion Control (Gravity, Shift, Impedance, Freedrive)
    # ==========================================================================

    async def save_gravity_parameter(
        self, robot_model: str, *, request: Request_Save_Gravity_ParameterPD
    ):
        req = Request_Save_Gravity_ParameterT()
        req.mode = request.mode
        req.gx = request.gx
        req.gy = request.gy
        req.gz = request.gz

        res = await self._execute_zenoh_query(
            f"{robot_model}/save_gravity_parameter", req, Response_FunctionsT, 16
        )
        return res["dict_payload"]


    async def set_shift(self, *, robot_model: str, request: Request_Set_ShiftPD):
        target_dict = request.target.model_dump()

        req = Request_Set_ShiftT()
        req.shiftNo = request.shift_no
        req.shiftMode = request.shift_mode
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target_dict["tar_values"]
        req.target.tarValues = ni
        req.target.tarFrame = target_dict["tar_frame"]
        req.target.tarUnit = target_dict["tar_unit"]

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_shift", req, Response_FunctionsT, 256
        )
        return res["dict_payload"]


    async def set_joint_impedance(self, *, robot_model: str, request: Request_Set_Joint_ImpedancePD):
        req = Request_Set_Joint_ImpedanceT()
        req.onoff = request.onoff

        req.stiffness = N_JOINT_fT()
        req.stiffness.f = request.stiffness.f
        
        req.torquelimit = N_JOINT_fT()
        req.torquelimit.f = request.torquelimit.f

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_joint_impedance", req, Response_FunctionsT, 64
        )
        return res["dict_payload"]


    async def set_freedrive(self, *, robot_model: str, request: Request_Set_Free_DrivePD):
        req = Request_Set_Free_DriveT()
        req.onoff = request.onoff
        req.sensitivity = request.sensitivity

        res = await self._execute_zenoh_query(
            f"{robot_model}/set_freedrive", req, Response_FunctionsT, 8
        )
        return res["dict_payload"]


    def call_reset_outcoll(self, robot_model: str):
        pass