from rb_flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from rb_flat_buffers.IPC.Request_CallConfigControlBox import Request_CallConfigControlBoxT
from rb_flat_buffers.IPC.Request_CallConfigRobotArm import Request_CallConfigRobotArmT
from rb_flat_buffers.IPC.Request_CallConfigToolList import Request_CallConfigToolListT
from rb_flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from rb_flat_buffers.IPC.Request_Save_Area_Para import Request_Save_Area_ParaT
from rb_flat_buffers.IPC.Request_Save_Collision_Parameter import Request_Save_Collision_ParameterT
from rb_flat_buffers.IPC.Request_Save_Direct_Teach_Sensitivity import (
    Request_Save_Direct_Teach_SensitivityT,
)
from rb_flat_buffers.IPC.Request_Save_Robot_Code import Request_Save_Robot_CodeT
from rb_flat_buffers.IPC.Request_Save_SelfColl_Parameter import Request_Save_SelfColl_ParameterT
from rb_flat_buffers.IPC.Request_Save_Tool_List_Para import Request_Save_Tool_List_ParaT
from rb_flat_buffers.IPC.Request_Save_User_Frame import Request_Save_User_FrameT
from rb_flat_buffers.IPC.Request_Set_Fixed_Speed_Mode import Request_Set_Fixed_Speed_ModeT
from rb_flat_buffers.IPC.Request_Set_Master_Mode import Request_Set_Master_ModeT
from rb_flat_buffers.IPC.Request_Set_Out_Collision_Para import Request_Set_Out_Collision_ParaT
from rb_flat_buffers.IPC.Request_Set_Self_Collision_Para import Request_Set_Self_Collision_ParaT
from rb_flat_buffers.IPC.Request_Set_Tool_List import Request_Set_Tool_ListT
from rb_flat_buffers.IPC.Request_Set_User_Frame import Request_Set_User_FrameT
from rb_flat_buffers.IPC.Request_Set_User_Frame_3Points import Request_Set_User_Frame_3PointsT
from rb_flat_buffers.IPC.Request_Set_User_Frame_6Dof import Request_Set_User_Frame_6DofT
from rb_flat_buffers.IPC.Request_Set_User_Frame_TCP import Request_Set_User_Frame_TCPT
from rb_flat_buffers.IPC.Response_CallConfigControlBox import Response_CallConfigControlBoxT
from rb_flat_buffers.IPC.Response_CallConfigRobotArm import Response_CallConfigRobotArmT
from rb_flat_buffers.IPC.Response_CallConfigToolList import Response_CallConfigToolListT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK
from .schema.manipulate_config_schema import (
    Request_Save_Area_ParameterSchema,
    Request_Save_Tool_ParameterSchema,
    Request_Save_User_Frame_ParameterSchema,
    Request_Set_User_Frame_3PointsSchema,
    Request_Set_User_Frame_6DofSchema,
)


class RBManipulateConfigSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Config SDK"""

    def set_toolist_num(
        self, *, robot_model: str, tool_num: int, flow_manager_args: FlowManagerArgs | None = None
    ):
        """TCP 번호 설정

        Args:
            robot_model: 로봇 모델명
            tool_num: TCP 번호
        """
        req = Request_Set_Tool_ListT()
        req.targetToolNum = tool_num

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_toollist_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Tool List Num failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_speedbar(
        self,
        *,
        robot_model: str,
        alpha: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Speed Bar 호출 함수]

        Args:
            robot_model: 로봇 모델명
            alpha: 스피드 바 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_MotionSpeedBarT()
        req.alpha = alpha

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_speedbar",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Speed Bar failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_config_robotarm(
        self,
        *,
        robot_model: str,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Config Robot Arm 호출 함수]

        Args:
            robot_model: 로봇 모델명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_CallConfigRobotArmT()

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_config_robotarm",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigRobotArmT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Config Robot Arm failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_config_controlbox(
        self,
        *,
        robot_model: str,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Config Control Box 호출 함수]

        Args:
            robot_model: 로봇 모델명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_CallConfigControlBoxT()
        res = self.zenoh_client.query_one(
            f"{robot_model}/call_config_controlbox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigControlBoxT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Config Control Box failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_robot_code(
        self,
        *,
        robot_model: str,
        code: str,
        option: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Robot Code 호출 함수]
        """
        req = Request_Save_Robot_CodeT()
        req.code = code
        req.option = option
        res = self.zenoh_client.query_one(
            f"{robot_model}/save_robot_code",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Robot Code failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_config_toollist(
        self,
        *,
        robot_model: str,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Config Tool List 호출 함수]

        Args:
            robot_model: 로봇 모델명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_CallConfigToolListT()
        res = self.zenoh_client.query_one(
            f"{robot_model}/call_config_toollist",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallConfigToolListT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Config Tool List failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_toollist_num(
        self,
        *,
        robot_model: str,
        target_tool_num: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set Tool List Num 호출 함수]
        """
        req = Request_Set_Tool_ListT()
        req.targetToolNum = target_tool_num

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_toollist_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Tool List Num failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_tool_list_parameter(
        self,
        *,
        robot_model: str,
        request: Request_Save_Tool_ParameterSchema,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Tool List Parameter 호출 함수]
        """
        req = Request_Save_Tool_List_ParaT()
        req.toolNo = request["tool_no"]
        req.toolName = request["tool_name"]
        req.tcpX = request["tcp_x"]
        req.tcpY = request["tcp_y"]
        req.tcpZ = request["tcp_z"]
        req.tcpRx = request["tcp_rx"]
        req.tcpRy = request["tcp_ry"]
        req.tcpRz = request["tcp_rz"]
        req.massM = request["mass_m"]
        req.massX = request["mass_x"]
        req.massY = request["mass_y"]
        req.massZ = request["mass_z"]
        req.boxType = request["box_type"]
        req.boxPara0 = request["box_para_0"]
        req.boxPara1 = request["box_para_1"]
        req.boxPara2 = request["box_para_2"]
        req.boxPara3 = request["box_para_3"]
        req.boxPara4 = request["box_para_4"]
        req.boxPara5 = request["box_para_5"]
        req.boxPara6 = request["box_para_6"]
        req.boxPara7 = request["box_para_7"]
        req.boxPara8 = request["box_para_8"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_tool_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=160,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Tool List Parameter failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]


    def set_userframe_num(
        self,
        *,
        robot_model: str,
        target_user_frame_num: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set User Frame Num 호출 함수]

        Args:
            robot_model: 로봇 모델명
            target_user_frame_num: 설정할 사용자 프레임 번호
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_User_FrameT()
        req.userFrameNum = target_user_frame_num

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_userframe_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set User Frame Num failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_user_frame_parameter(
        self,
        *,
        robot_model: str,
        request: Request_Save_User_Frame_ParameterSchema,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save User Frame Parameter 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 설정할 사용자 프레임 파라미터 설정 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Save_User_FrameT()
        req.userfNo = request["userf_no"]
        req.userfName = request["userf_name"]
        req.userfX = request["userf_x"]
        req.userfY = request["userf_y"]
        req.userfZ = request["userf_z"]
        req.userfRx = request["userf_rx"]
        req.userfRy = request["userf_ry"]
        req.userfRz = request["userf_rz"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_user_frame_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save User Frame Parameter failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_userframe_6dof(
        self,
        *,
        robot_model: str,
        request: Request_Set_User_Frame_6DofSchema,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set User Frame 6Dof 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 설정할 사용자 프레임 6Dof 설정 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_User_Frame_6DofT()
        req.userFrameNum = request["user_frame_num"]
        req.settingOption = request["setting_option"]
        req.targetX = request["target_x"]
        req.targetY = request["target_y"]
        req.targetZ = request["target_z"]
        req.targetRx = request["target_rx"]
        req.targetRy = request["target_ry"]
        req.targetRz = request["target_rz"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_userframe_6dof",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set User Frame 6Dof failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_userframe_tcp(
        self,
        *,
        robot_model: str,
        user_frame_num: int,
        setting_option: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """ [Set User Frame TCP 호출 함수]

        Args:
            robot_model: 로봇 모델명
            user_frame_num: 설정할 사용자 프레임 번호
            setting_option: 설정할 사용자 프레임 TCP 설정 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_User_Frame_TCPT()
        req.userFrameNum = user_frame_num
        req.settingOption = setting_option

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_userframe_tcp",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set User Frame TCP failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]


    def set_userframe_3points(
        self,
        *,
        robot_model: str,
        request: Request_Set_User_Frame_3PointsSchema,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set User Frame 3Points 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 설정할 사용자 프레임 3Points 설정 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_User_Frame_3PointsT()
        req.userFrameNum = request["user_frame_num"]
        req.settingOption = request["setting_option"]
        req.orderOption = request["order_option"]
        req.point1X = request["point_1_x"]
        req.point1Y = request["point_1_y"]
        req.point1Z = request["point_1_z"]
        req.point2X = request["point_2_x"]
        req.point2Y = request["point_2_y"]
        req.point2Z = request["point_2_z"]
        req.point3X = request["point_3_x"]
        req.point3Y = request["point_3_y"]
        req.point3Z = request["point_3_z"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_userframe_3points",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=48,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set User Frame 3Points failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_area_parameter(
        self,
        *,
        robot_model: str,
        request: Request_Save_Area_ParameterSchema,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Area Parameter 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 설정할 영역 파라미터 설정 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Save_Area_ParaT()
        req.areaNo = request["area_no"]
        req.areaName = request["area_name"]
        req.areaType = request["area_type"]
        req.areaX = request["area_x"]
        req.areaY = request["area_y"]
        req.areaZ = request["area_z"]
        req.areaRx = request["area_rx"]
        req.areaRy = request["area_ry"]
        req.areaRz = request["area_rz"]
        req.areaPara0 = request["area_para_0"]
        req.areaPara1 = request["area_para_1"]
        req.areaPara2 = request["area_para_2"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_area_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=100,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Area Parameter failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_direct_teach_sensitivity(
        self,
        *,
        robot_model: str,
        sensitivity: list[float],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Direct Teach Sensitivity 호출 함수]

        Args:
            robot_model: 로봇 모델명
            sensitivity: 설정할 직접 교육 감도 값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Save_Direct_Teach_SensitivityT()
        nj = N_JOINT_fT()
        nj.f = [float(x) for x in sensitivity]
        req.sensitivity = nj

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_direct_teach_sensitivity",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Direct Teach Sensitivity failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_collision_parameter(
        self,
        *,
        robot_model: str,
        onoff: int,
        react: int,
        threshold: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Collision Parameter 호출 함수]

        Args:
            robot_model: 로봇 모델명
            onoff: 충돌 감지 온/오프 여부
            react: 충돌 반응 모드
            threshold: 충돌 감지 임계값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Save_Collision_ParameterT()
        req.onoff = onoff
        req.react = react
        req.threshold = threshold

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_collision_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=12,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Collision Parameter failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def save_selfcoll_parameter(
        self,
        *,
        robot_model: str,
        mode: int,
        dist_internal: float,
        dist_external: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Save Self Collision Parameter 호출 함수]

        Args:
            robot_model: 로봇 모델명
            mode: 충돌 감지 모드
            dist_internal: 충돌 감지 내부 거리
            dist_external: 충돌 감지 외부 거리
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Save_SelfColl_ParameterT()
        req.mode = mode
        req.distInternal = dist_internal
        req.distExternal = dist_external

        res = self.zenoh_client.query_one(
            f"{robot_model}/save_selfcoll_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=12,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Save Self Collision Parameter failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_out_collision_para(
        self,
        *,
        robot_model: str,
        onoff: int,
        react_mode: int,
        threshold: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set Out Collision Para 호출 함수]

        Args:
            robot_model: 로봇 모델명
            onoff: 충돌 감지 온/오프 여부
            react_mode: 충돌 반응 모드
            threshold: 충돌 감지 임계값
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_Out_Collision_ParaT()
        req.onoff = onoff
        req.reactMode = react_mode
        req.threshold = threshold

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_out_collision_para",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=12,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Out Collision Para failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_self_collision_para(
        self,
        *,
        robot_model: str,
        mode: int,
        dist_int: float,
        dist_ext: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set Self Collision Para 호출 함수]

        Args:
            robot_model: 로봇 모델명
            mode: 충돌 감지 모드
            dist_int: 충돌 감지 내부 거리
            dist_ext: 충돌 감지 외부 거리
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_Self_Collision_ParaT()
        req.mode = mode
        req.distInt = dist_int
        req.distExt = dist_ext

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_self_collision_para",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=12,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Self Collision Para failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_master_mode(
        self,
        *,
        robot_model: str,
        mode: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set Master Mode 호출 함수]

        Args:
            robot_model: 로봇 모델명
            mode: 마스터 모드
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_Master_ModeT()
        req.mode = mode

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_master_mode",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Master Mode failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def set_fixed_speed(
        self,
        *,
        robot_model: str,
        target_move: int,
        vel_option: int,
        vel_parameter: float,
        acc_option: int,
        acc_parameter: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """
        [Set Fixed Speed 호출 함수]

        Args:
            robot_model: 로봇 모델명
            target_move: 이동 타입 (0: J-Types, 1: L-Types)
            vel_option: 속도 옵션 (0: Off, 1: Fixed mode: %, 2: Fixed mode: Physical)
            vel_parameter: 속도 파라미터
            acc_option: 가속도 옵션 (0: Off, 1: Fixed mode: %, 2: Fixed mode: Physical)
            acc_parameter: 가속도 파라미터 (mm/s^2 또는 deg/s^2)
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Set_Fixed_Speed_ModeT()
        req.targetMove = target_move
        req.velOption = vel_option
        req.velParameter = vel_parameter
        req.accOption = acc_option
        req.accParameter = acc_parameter

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_fixed_speed",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=16,
            timeout=3.0,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Set Fixed Speed failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]
