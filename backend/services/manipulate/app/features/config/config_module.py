from app.socket.socket_client import socket_client  # pyright: ignore
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from rb_flat_buffers.IPC.Request_Save_Gravity_Parameter import Request_Save_Gravity_ParameterT
from rb_flat_buffers.IPC.Request_Set_Free_Drive import Request_Set_Free_DriveT
from rb_flat_buffers.IPC.Request_Set_Joint_Impedance import Request_Set_Joint_ImpedanceT
from rb_flat_buffers.IPC.Request_Set_Shift import Request_Set_ShiftT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_schemas.fbs_models.manipulate.v1.func_arm_models import Request_Save_Tool_List_ParaPD
from rb_schemas.fbs_models.manipulate.v1.func_config_models import (
    Response_CallConfigControlBoxPD,
    Response_CallConfigRobotArmPD,
    Response_CallConfigToolListPD,
)
from rb_schemas.fbs_models.manipulate.v1.func_return_models import Response_FunctionsPD
from rb_schemas.fbs_models.manipulate.v1.func_set_models import (
    Request_Set_Fixed_Speed_ModePD,
    Request_Set_Master_ModePD,
    Request_Set_Out_Collision_ParaPD,
    Request_Set_Self_Collision_ParaPD,
    Request_Set_User_Frame_TCPPD,
)
from rb_sdk.manipulate_sdk.manipulate_config import RBManipulateConfigSDK
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
    Request_Save_User_FramePD,
    Request_Set_Free_DrivePD,
    Request_Set_Joint_ImpedancePD,
    Request_Set_ShiftPD,
    Request_Set_Tool_ListPD,
    Request_Set_User_Frame_3PointsPD,
    Request_Set_User_Frame_6DofPD,
    Request_Set_User_FramePD,
)

zenoh_client = ZenohClient()
manipulate_config_sdk = RBManipulateConfigSDK()


class ConfigService(BaseService):
    """ Config Service """
    # ==========================================================================
    # 1. General Configuration (Robot Arm, Control Box, Code)
    # ==========================================================================

    def call_config_robotarm(self, robot_model: str):
        """ Config Robot Arm Service """
        res = manipulate_config_sdk.call_config_robotarm(robot_model=robot_model)

        return Response_CallConfigRobotArmPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def call_config_controlbox(self, robot_model: str):
        """ Config Control Box Service """
        res = manipulate_config_sdk.call_config_controlbox(robot_model=robot_model)

        return Response_CallConfigControlBoxPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def save_robot_code(self, robot_model: str, request: Request_Save_Robot_CodePD):
        """ Save Robot Code Service """
        res = manipulate_config_sdk.save_robot_code(
            robot_model=robot_model,
            code=request.code,
            option=request.option,
        )

        return Response_FunctionsPD.model_validate(t_to_dict(res)).model_dump(by_alias=True)


    def socket_emit_config_robot_arm(self, robot_model: str):
        """ Socket Emit Config Robot Arm Service """
        config_robot_arm_res = self.call_config_robotarm(robot_model)

        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_robotarm", to_json(config_robot_arm_res))
        )


    def socket_emit_config_control_box(
        self,
        robot_model: str,
        *,
        emit_user_frames: bool = False,
    ):
        """ Socket Emit Config Control Box Service """
        config_control_box_res = self.call_config_controlbox(robot_model)

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

    def call_config_toollist(self, robot_model: str):
        """ Call Config Tool List Service """
        res = manipulate_config_sdk.call_config_toollist(robot_model=robot_model)

        return Response_CallConfigToolListPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def set_toollist_num(self, robot_model: str, *, request: Request_Set_Tool_ListPD):
        """ Set Tool List Num Service """
        res = manipulate_config_sdk.set_toollist_num(
            robot_model=robot_model,
            target_tool_num=request.target_tool_num,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def save_tool_list_parameter(
        self, robot_model: str, *, request: Request_Save_Tool_List_ParaPD
    ):
        """ Save Tool List Parameter Service """
        res = manipulate_config_sdk.save_tool_list_parameter(
            robot_model=robot_model,
            request=request.model_dump(),
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def socket_emit_config_toollist(self, robot_model: str):
        """ Socket Emit Config Tool List Service """
        config_toollist_res = self.call_config_toollist(robot_model)

        fire_and_log(
            socket_client.emit(f"{robot_model}/call_config_toollist", to_json(config_toollist_res))
        )

    # ==========================================================================
    # 3. Coordinate & Area Configuration
    # ==========================================================================

    def get_user_frames(self, robot_model: str):
        """ Get User Frames Service """
        res = self.call_config_controlbox(robot_model)

        user_frames_res = self.parse_get_user_frames(res)
        return user_frames_res


    def parse_get_user_frames(self, config_control_box_res: Response_CallConfigControlBoxPD):
        """ Parse Get User Frames Service """
        res_data = t_to_dict(config_control_box_res)
        return {
            "user_frames": [
                res_data.get(f"userFrame{i}") for i in range(8)
            ]
        }


    def set_userframe_num(self, robot_model: str, *, request: Request_Set_User_FramePD):
        """ Set User Frame Num Service """
        res = manipulate_config_sdk.set_userframe_num(
            robot_model=robot_model,
            target_user_frame_num=request.user_frame_num,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def save_user_frame_parameter(
        self,
        robot_model: str,
        *,
        request: Request_Save_User_FramePD,
    ):
        """ Save User Frame Parameter Service """
        res = manipulate_config_sdk.save_user_frame_parameter(
            robot_model=robot_model,
            request=request.model_dump(),
        )

        self.socket_emit_config_control_box(robot_model, emit_user_frames=True)

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def set_userframe_6dof(
        self,
        *,
        robot_model: str,
        request: Request_Set_User_Frame_6DofPD,
    ):
        """ Set User Frame 6Dof Service """
        res = manipulate_config_sdk.set_userframe_6dof(
            robot_model=robot_model,
            request=request.model_dump(),
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def set_userframe_tcp(self, *, robot_model: str, request: Request_Set_User_Frame_TCPPD):
        """ Set User Frame TCP Service """
        res = manipulate_config_sdk.set_userframe_tcp(
            robot_model=robot_model,
            user_frame_num=request.user_frame_num,
            setting_option=request.setting_option,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    async def set_userframe_3points(
        self,
        *,
        robot_model: str,
        request: Request_Set_User_Frame_3PointsPD,
    ):
        """ Set User Frame 3Points Service """
        res = manipulate_config_sdk.set_userframe_3points(
            robot_model=robot_model,
            request=request.model_dump(),
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    def save_area_parameter(
        self,
        robot_model: str,
        *,
        request: Request_Save_Area_ParameterPD,
    ):
        """ Save Area Parameter Service """
        res = manipulate_config_sdk.save_area_parameter(
            robot_model=robot_model,
            request=request.model_dump(),
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)


    # ==========================================================================
    # 4. Safety & Sensitivity
    # ==========================================================================

    def save_direct_teach_sensitivity(
        self,
        robot_model: str,
        *,
        request: Request_Save_Direct_Teach_SensitivityPD,
    ):
        """ Save Direct Teach Sensitivity Service """
        res = manipulate_config_sdk.save_direct_teach_sensitivity(
            robot_model=robot_model,
            sensitivity=request.sensitivity,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

    def save_collision_parameter(
        self,
        robot_model: str,
        *,
        request: Request_Save_Collision_ParameterPD,
    ):
        """ Save Collision Parameter Service """
        res = manipulate_config_sdk.save_collision_parameter(
            robot_model=robot_model,
            onoff=request.onoff,
            react=request.react,
            threshold=request.threshold,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

    async def save_selfcoll_parameter(
        self,
        *,
        robot_model: str,
        request: Request_Save_SelfColl_ParameterPD,
    ):
        """ Save Self Collision Parameter Service """
        res = manipulate_config_sdk.save_selfcoll_parameter(
            robot_model=robot_model,
            mode=request.mode,
            dist_internal=request.dist_internal,
            dist_external=request.dist_external,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

    async def set_out_collision_para(
        self,
        *,
        robot_model: str,
        request: Request_Set_Out_Collision_ParaPD
    ):
        """ Set Out Collision Para Service """
        res = manipulate_config_sdk.set_out_collision_para(
            robot_model=robot_model,
            onoff=request.onoff,
            react_mode=request.react_mode,
            threshold=request.threshold,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

    async def set_self_collision_para(
        self,
        *,
        robot_model: str,
        request: Request_Set_Self_Collision_ParaPD,
    ):
        """ Set Self Collision Para Service """
        res = manipulate_config_sdk.set_self_collision_para(
            robot_model=robot_model,
            mode=request.mode,
            dist_int=request.dist_int,
            dist_ext=request.dist_ext,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

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

        res = zenoh_client.query_one(
            f"{robot_model}/save_gravity_parameter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=16,
        )
        return t_to_dict(res)["dict_payload"]


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

        res = zenoh_client.query_one(
            f"{robot_model}/set_shift",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )
        return t_to_dict(res)["dict_payload"]


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
            flatbuffer_buf_size=64,
        )
        return t_to_dict(res)["dict_payload"]


    async def set_freedrive(self, *, robot_model: str, request: Request_Set_Free_DrivePD):
        req = Request_Set_Free_DriveT()
        req.onoff = request.onoff
        req.sensitivity = request.sensitivity

        res = zenoh_client.query_one(
            f"{robot_model}/set_freedrive",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return t_to_dict(res)["dict_payload"]


    def call_reset_outcoll(self, robot_model: str):
        pass

    def set_master_mode(
        self,
        *,
        robot_model: str,
        request: Request_Set_Master_ModePD,
    ):
        """ Set Master Mode Service """
        res = manipulate_config_sdk.set_master_mode(
            robot_model=robot_model,
            mode=request.mode,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)

    def set_fixed_speed(
        self,
        *,
        robot_model: str,
        request: Request_Set_Fixed_Speed_ModePD,
    ):
        """ Set Fixed Speed Service """
        res = manipulate_config_sdk.set_fixed_speed(
            robot_model=robot_model,
            target_move=request.target_move,
            vel_option=request.vel_option,
            vel_parameter=request.vel_parameter,
            acc_option=request.acc_option,
            acc_parameter=request.acc_parameter,
        )

        return Response_FunctionsPD.model_validate(
            t_to_dict(res)
        ).model_dump(by_alias=True)
