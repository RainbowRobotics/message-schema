from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.MoveInput_Type import MoveInput_TypeT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_Move_JB_ADD import Request_Move_JB_ADDT
from rb_flat_buffers.IPC.Request_Move_JB_CLR import Request_Move_JB_CLRT
from rb_flat_buffers.IPC.Request_Move_JB_RUN import Request_Move_JB_RUNT
from rb_flat_buffers.IPC.Request_Move_LB_ADD import Request_Move_LB_ADDT
from rb_flat_buffers.IPC.Request_Move_LB_CLR import Request_Move_LB_CLRT
from rb_flat_buffers.IPC.Request_Move_LB_RUN import Request_Move_LB_RUNT
from rb_flat_buffers.IPC.Request_Move_XB_ADD import Request_Move_XB_ADDT
from rb_flat_buffers.IPC.Request_Move_XB_CLR import Request_Move_XB_CLRT
from rb_flat_buffers.IPC.Request_Move_XB_RUN import Request_Move_XB_RUNT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_sdk.manipulate import RBManipulateSDK
from rb_sdk.manipulate_sdk.schema.manipulate_move_schema import MoveInputSpeedSchema
from rb_sdk.schema.manipulate_schema import MoveInputTargetSchema
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from .program_schema import (
    Request_MotionSpeedBarPD,
    Request_Move_SmoothJogJPD,
    Request_Move_SmoothJogLPD,
    Request_Move_SmoothJogStopPD,
    Request_MoveJBAddPD,
    Request_MoveJPD,
    Request_MoveLBAddPD,
    Request_MoveLBRunPD,
    Request_MoveLPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
    Request_MoveXBAddPD,
    Request_MoveXBRunPD,
    Request_ProgramAfterPD,
    Request_ProgramBeforePD,
    Request_RelativeMovePD,
)

zenoh_client = ZenohClient()
manipulate_sdk = RBManipulateSDK()


class ProgramService(BaseService):
    """Program Service"""
    # ==========================================================================
    # 1. System Control (Resume, Pause, Halt, Program Flow)
    # ==========================================================================

    async def call_resume(self, *, robot_model: str):
        """
        [Resume 호출]
        """

        res = manipulate_sdk.program.call_resume(
            robot_model=robot_model,
        )

        return t_to_dict(res)


    async def call_pause(self, *, robot_model: str):
        """
        [Pause 호출]
        """

        res = manipulate_sdk.program.call_pause(
            robot_model=robot_model,
        )

        return t_to_dict(res)


    async def call_halt(self, *, robot_model: str):
        """
        [Halt 호출]
        """

        res = manipulate_sdk.program.call_halt(
            robot_model=robot_model,
        )

        return t_to_dict(res)


    def call_program_before(self, *, robot_model: str, request : Request_ProgramBeforePD):
        """
        [Program Before 호출]
        - option: 프로그램 옵션
        """

        res = manipulate_sdk.program.call_program_before(
            robot_model=robot_model,
            option=request.option
        )

        return t_to_dict(res)


    def call_program_after(self, *, robot_model: str, request : Request_ProgramAfterPD):
        """
        [Program After 호출]
        - option: 프로그램 옵션
        """

        res = manipulate_sdk.program.call_program_after(
            robot_model=robot_model,
            option=request.option,
        )

        return t_to_dict(res)


    def call_speedbar(self, *, robot_model: str, request : Request_MotionSpeedBarPD):
        """
        [Speed Bar 호출]
        - request: Speed Bar 요청 데이터 (alpha: 스피드 바 값)
        """

        res = manipulate_sdk.config.call_speedbar(
            robot_model=robot_model,
            alpha=request.alpha,
        )

        return t_to_dict(res)

    # ==========================================================================
    # 2. Jog Control (SmoothJog, TickJog)
    # ==========================================================================

    def call_smoothjog_j(
        self, *, robot_model: str, request: Request_Move_SmoothJogJPD
    ):
        """
        [Smooth Jog J 호출]
        - target: 이동 좌표 및 프레임 정보
        """

        res = manipulate_sdk.move.call_smoothjog_j(
            robot_model=robot_model,
            target=MoveInputTargetSchema(
                tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ),
        )

        return t_to_dict(res)


    def call_smoothjog_l(
        self, *, robot_model: str, request: Request_Move_SmoothJogLPD
    ):
        """
        [Smooth Jog L 호출]
        - target: 이동 좌표 및 프레임 정보
        """

        res = manipulate_sdk.move.call_smoothjog_l(
            robot_model=robot_model,
            target=MoveInputTargetSchema(
                tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ),
        )

        return t_to_dict(res)


    def call_smoothjog_stop(self, *, robot_model: str, request: Request_Move_SmoothJogStopPD):
        """
        [Smooth Jog Stop 호출]
        - stop_time: 정지 시간
        """

        res = manipulate_sdk.move.call_smoothjog_stop(
            robot_model=robot_model,
            stop_time=request.stoptime,
        )

        return t_to_dict(res)


    def call_tickjog_j(self, *, robot_model: str, request: Request_MoveTickJogJPD):
        """
        [Tick Jog J 호출]
        - target: 이동 좌표 및 프레임 정보
        - speed: 이동 속도 정보
        """

        res = manipulate_sdk.move.call_tickjog_j(
            robot_model=robot_model,
            target=MoveInputTargetSchema(
                tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ),
            speed=MoveInputSpeedSchema(
                spd_mode=request.speed.spd_mode,
                spd_vel_para=request.speed.spd_vel_para,
                spd_acc_para=request.speed.spd_acc_para,
            ),
        )

        return t_to_dict(res)


    async def call_tickjog_l(self, *, robot_model: str, request: Request_MoveTickJogLPD):
        """
        [Tick Jog L 호출]
        - target: 이동 좌표 및 프레임 정보
        - speed: 이동 속도 정보
        """

        res = manipulate_sdk.move.call_tickjog_l(
            robot_model=robot_model,
            target=MoveInputTargetSchema(
            tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ),
            speed=MoveInputSpeedSchema(
                spd_mode=request.speed.spd_mode,
                spd_vel_para=request.speed.spd_vel_para,
                spd_acc_para=request.speed.spd_acc_para,
            ),
        )

        return t_to_dict(res)

    # ==========================================================================
    # 3. Direct Move Control (MoveJ, MoveL)
    # ==========================================================================

    def call_move_j(self, *, robot_model: str, request: Request_MoveJPD):
        """
        [Move J 호출]
        - target: 이동 좌표 및 프레임 정보
        - speed: 이동 속도 정보
        """

        res = manipulate_sdk.move.call_move_j(
            robot_model=robot_model,
            target=MoveInputTargetSchema(
                tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ),
            speed=MoveInputSpeedSchema(
                spd_mode=request.speed.spd_mode,
                spd_vel_para=request.speed.spd_vel_para,
                spd_acc_para=request.speed.spd_acc_para,
            ),
        )

        return t_to_dict(res)

    def call_move_l(self, *, robot_model: str, request: Request_MoveLPD):
        """
        [Move L 호출]
        - target: 이동 좌표 및 프레임 정보
        - speed: 이동 속도 정보
        """

        res = manipulate_sdk.move.call_move_l(
                robot_model=robot_model,
                target=MoveInputTargetSchema(
                tar_values=request.target.tar_values,
                tar_frame=request.target.tar_frame,
                tar_unit=request.target.tar_unit,
            ), speed=MoveInputSpeedSchema(
                spd_mode=request.speed.spd_mode,
                spd_vel_para=request.speed.spd_vel_para,
                spd_acc_para=request.speed.spd_acc_para,
            ),
        )

        return t_to_dict(res)

    # ==========================================================================
    # 4. Blend Move Control (JB, LB, XB)
    # ==========================================================================

    async def call_move_jb_clr(self, *, robot_model: str):
        req = Request_Move_JB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_move_jb_add(self, *, robot_model: str, request: Request_MoveJBAddPD):
        req = Request_Move_JB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        req.target.tarValues = ni
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        req.speed.spdMode = request.speed.spd_mode
        req.speed.spdVelPara = request.speed.spd_vel_para
        req.speed.spdAccPara = request.speed.spd_acc_para

        req.type.pntType = request.type.pnt_type
        req.type.pntPara = request.type.pnt_para

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_move_jb_run(self, *, robot_model: str):
        req = Request_Move_JB_RUNT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_move_lb_clr(self, *, robot_model: str):
        req = Request_Move_LB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_move_lb_add(self, *, robot_model: str, request: Request_MoveLBAddPD):
        req = Request_Move_LB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        req.target.tarValues = ni
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        req.speed.spdMode = request.speed.spd_mode
        req.speed.spdVelPara = request.speed.spd_vel_para
        req.speed.spdAccPara = request.speed.spd_acc_para

        req.type.pntType = request.type.pnt_type
        req.type.pntPara = request.type.pnt_para

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_move_lb_run(self, *, robot_model: str, request: Request_MoveLBRunPD):
        req = Request_Move_LB_RUNT()
        req.orientation = request.orientation

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_move_xb_clr(self, *, robot_model: str):
        req = Request_Move_XB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_move_xb_add(self, *, robot_model: str, request: Request_MoveXBAddPD):
        req = Request_Move_XB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        req.target.tarValues = ni
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        req.speed.spdMode = request.speed.spd_mode
        req.speed.spdVelPara = request.speed.spd_vel_para
        req.speed.spdAccPara = request.speed.spd_acc_para

        req.type.pntType = request.type.pnt_type
        req.type.pntPara = request.type.pnt_para

        req.method = request.method

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_move_xb_run(self, *, robot_model: str, request: Request_MoveXBRunPD):
        req = Request_Move_XB_RUNT()
        req.runningMode = request.running_mode

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def call_relative_move(self, *, robot_model: str, request: Request_RelativeMovePD):
        """
        [Relative Move 호출]
        - relative_value: 상대 좌표
        - reference_value: 기준 좌표
        - move_type: 이동 타입 (0: move J 계열, 1: move L 계열)
        - speed: 속도
        """
        res = manipulate_sdk.move.call_relative_move(
            robot_model=robot_model,
            relative_value=MoveInputTargetSchema(
                tar_values=request.relative_value.tar_values,
                tar_frame=request.relative_value.tar_frame,
                tar_unit=request.relative_value.tar_unit,
            ),
            reference_value=request.reference_value,
            move_type=request.move_type,
            speed=request.speed,
        )

        return t_to_dict(res)
