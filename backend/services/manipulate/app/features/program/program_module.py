from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.MoveInput_Type import MoveInput_TypeT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_MotionResume import Request_MotionResumeT
from rb_flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from rb_flat_buffers.IPC.Request_Move_J import Request_Move_JT
from rb_flat_buffers.IPC.Request_Move_JB_ADD import Request_Move_JB_ADDT
from rb_flat_buffers.IPC.Request_Move_JB_CLR import Request_Move_JB_CLRT
from rb_flat_buffers.IPC.Request_Move_JB_RUN import Request_Move_JB_RUNT
from rb_flat_buffers.IPC.Request_Move_L import Request_Move_LT
from rb_flat_buffers.IPC.Request_Move_LB_ADD import Request_Move_LB_ADDT
from rb_flat_buffers.IPC.Request_Move_LB_CLR import Request_Move_LB_CLRT
from rb_flat_buffers.IPC.Request_Move_LB_RUN import Request_Move_LB_RUNT
from rb_flat_buffers.IPC.Request_Move_SmoothJogJ import Request_Move_SmoothJogJT
from rb_flat_buffers.IPC.Request_Move_SmoothJogL import Request_Move_SmoothJogLT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Request_Move_TickJogJ import Request_Move_TickJogJT
from rb_flat_buffers.IPC.Request_Move_TickJogL import Request_Move_TickJogLT
from rb_flat_buffers.IPC.Request_Move_XB_ADD import Request_Move_XB_ADDT
from rb_flat_buffers.IPC.Request_Move_XB_CLR import Request_Move_XB_CLRT
from rb_flat_buffers.IPC.Request_Move_XB_RUN import Request_Move_XB_RUNT
from rb_flat_buffers.IPC.Request_ProgramAfter import Request_ProgramAfterT
from rb_flat_buffers.IPC.Request_ProgramBefore import Request_ProgramBeforeT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_sdk.manipulate import RBManipulateSDK
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
    def __init__(self):
        pass

    # ==========================================================================
    # 1. System Control (Resume, Pause, Halt, Program Flow)
    # ==========================================================================

    async def call_resume(self, *, robot_model: str):
        req = Request_MotionResumeT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return res["dict_payload"]


    async def call_pause(self, *, robot_model: str):
        req = Request_MotionPauseT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return res["dict_payload"]


    async def call_halt(self, *, robot_model: str):
        req = Request_MotionHaltT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_halt",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return res["dict_payload"]


    async def call_program_before(self, *, robot_model: str, request : Request_ProgramBeforePD):
        req = Request_ProgramBeforeT()
        req.option = request.option

        res = zenoh_client.query_one(
            f"{robot_model}/call_program_before",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return res["dict_payload"]


    async def call_program_after(self, *, robot_model: str, request : Request_ProgramAfterPD):
        req = Request_ProgramAfterT()
        req.option = request.option

        res = zenoh_client.query_one(
            f"{robot_model}/call_program_after",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return res["dict_payload"]


    async def call_speedbar(self, *, robot_model: str, request : Request_MotionSpeedBarPD):
        req = Request_MotionSpeedBarT()
        req.alpha = request.alpha

        res = zenoh_client.query_one(
            f"{robot_model}/call_speedbar",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    # ==========================================================================
    # 2. Jog Control (SmoothJog, TickJog)
    # ==========================================================================

    async def call_smoothjog_j(
        self, *, robot_model: str, request: Request_Move_SmoothJogJPD
    ):
        req = Request_Move_SmoothJogJT()
        req.target = MoveInput_TargetT()

        nf = N_INPUT_fT()
        nf.f = request.target.tar_values

        req.target.tarValues = nf
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def call_smoothjog_l(
        self, *, robot_model: str, request: Request_Move_SmoothJogLPD
    ):
        req = Request_Move_SmoothJogLT()
        req.target = MoveInput_TargetT()

        nf = N_INPUT_fT()
        nf.f = request.target.tar_values

        req.target.tarValues = nf
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def call_smoothjog_stop(self, *, robot_model: str, request: Request_Move_SmoothJogStopPD):
        req = Request_Move_SmoothJogStopT()
        req.stoptime = request.stoptime

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]


    async def call_tickjog_j(self, *, robot_model: str, request: Request_MoveTickJogJPD):
        req = Request_Move_TickJogJT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        req.target.tarValues = ni
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        req.speed.spdMode = request.speed.spd_mode
        req.speed.spdVelPara = request.speed.spd_vel_para
        req.speed.spdAccPara = request.speed.spd_acc_para

        res = zenoh_client.query_one(
            f"{robot_model}/call_tickjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def call_tickjog_l(self, *, robot_model: str, request: Request_MoveTickJogLPD):
        req = Request_Move_TickJogLT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        req.target.tarValues = ni
        req.target.tarFrame = request.target.tar_frame
        req.target.tarUnit = request.target.tar_unit

        req.speed.spdMode = request.speed.spd_mode
        req.speed.spdVelPara = request.speed.spd_vel_para
        req.speed.spdAccPara = request.speed.spd_acc_para

        res = zenoh_client.query_one(
            f"{robot_model}/call_tickjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    # ==========================================================================
    # 3. Direct Move Control (MoveJ, MoveL)
    # ==========================================================================

    async def call_move_j(self, *, robot_model: str, request: Request_MoveJPD):
        req = Request_Move_JT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        move_input_target.tarValues = ni
        move_input_target.tarFrame = request.target.tar_frame
        move_input_target.tarUnit = request.target.tar_unit

        move_input_speed.spdMode = request.speed.spd_mode
        move_input_speed.spdVelPara = request.speed.spd_vel_para
        move_input_speed.spdAccPara = request.speed.spd_acc_para

        req.target = move_input_target
        req.speed = move_input_speed

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]


    async def call_move_l(self, *, robot_model: str, request: Request_MoveLPD):
        req = Request_Move_LT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target.tar_values

        move_input_target.tarValues = ni
        move_input_target.tarFrame = request.target.tar_frame

        move_input_speed.spdMode = request.speed.spd_mode
        move_input_speed.spdVelPara = request.speed.spd_vel_para
        move_input_speed.spdAccPara = request.speed.spd_acc_para

        req.target = move_input_target
        req.speed = move_input_speed

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

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
        req_dict = request.model_dump()
        return await manipulate_sdk.relative_move(
            robot_model=robot_model,
            relative_value=req_dict.get("relative_value"),
            reference_value=req_dict.get("reference_value"),
            move_type=req_dict.get("move_type"),
            spd_mode=req_dict.get("speed").get("spd_mode"),
            spd_vel_para=req_dict.get("speed").get("spd_vel_para"),
            spd_acc_para=req_dict.get("speed").get("spd_acc_para"),
        )
