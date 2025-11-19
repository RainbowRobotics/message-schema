from app.features.program.program_schema import MoveInputSpeed, MoveInputTarget
from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.MoveInput_Type import MoveInput_TypeT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
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
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from .program_schema import (
    Request_MoveJBAddPD,
    Request_MoveLBAddPD,
    Request_MoveLBRunPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
    Request_MoveXBAddPD,
    Request_MoveXBRunPD,
)

zenoh_client = ZenohClient()


class ProgramService(BaseService):
    def __init__(self):
        pass

    async def call_resume(self, *, robot_model: str):
        req = Request_MotionPauseT()

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

    async def call_speedbar(self, *, robot_model: str, speedbar: float):
        req = Request_MotionSpeedBarT()
        req.alpha = speedbar

        res = zenoh_client.query_one(
            f"{robot_model}/call_speedbar",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    async def call_smoothjog_j(
        self, *, robot_model: str, targetspeed: list[float], frame: int, unit: int
    ):
        req = Request_Move_SmoothJogJT()
        req.target = MoveInput_TargetT()

        nf = N_INPUT_fT()
        nf.f = targetspeed
        req.target.tarValues = nf

        req.target.tarFrame = frame
        req.target.tarUnit = unit

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_smoothjog_l(
        self, *, robot_model: str, targetspeed: list[float], frame: int, unit: int
    ):
        req = Request_Move_SmoothJogLT()
        req.target = MoveInput_TargetT()

        nf = N_INPUT_fT()
        nf.f = targetspeed
        req.target.tarValues = nf

        req.target.tarFrame = frame
        req.target.tarUnit = unit

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_smoothjog_stop(self, *, robot_model: str, stoptime: float):
        req = Request_Move_SmoothJogStopT()
        req.stoptime = stoptime

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    async def call_move_j(
        self, *, robot_model: str, target: MoveInputTarget, speed: MoveInputSpeed
    ):
        target_dict = t_to_dict(target)
        speed_dict = t_to_dict(speed)

        req = Request_Move_JT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target_dict["tar_values"]

        move_input_target.tarValues = ni
        move_input_target.tarFrame = target_dict["tar_frame"]
        move_input_target.tarUnit = target_dict["tar_unit"]

        move_input_speed.spdMode = speed_dict["spd_mode"]
        move_input_speed.spdVelPara = speed_dict["spd_vel_para"]
        move_input_speed.spdAccPara = speed_dict["spd_acc_para"]

        req.target = move_input_target
        req.speed = move_input_speed

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_move_l(
        self, *, robot_model: str, target: MoveInputTarget, speed: MoveInputSpeed
    ):
        target_dict = t_to_dict(target)
        speed_dict = t_to_dict(speed)

        req = Request_Move_LT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target_dict["tar_values"]

        move_input_target.tarValues = ni
        move_input_target.tarFrame = target_dict["tar_frame"]
        move_input_target.tarUnit = target_dict["tar_unit"]

        move_input_speed.spdMode = speed_dict["spd_mode"]
        move_input_speed.spdVelPara = speed_dict["spd_vel_para"]
        move_input_speed.spdAccPara = speed_dict["spd_acc_para"]

        req.target = move_input_target
        req.speed = move_input_speed

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_tickjog_j(self, *, robot_model: str, request: Request_MoveTickJogJPD):
        target = t_to_dict(request.target)
        speed = t_to_dict(request.speed)

        req = Request_Move_TickJogJT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed["spd_mode"]
        req.speed.spdVelPara = speed["spd_vel_para"]
        req.speed.spdAccPara = speed["spd_acc_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_tickjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_tickjog_l(self, *, robot_model: str, request: Request_MoveTickJogLPD):
        target = t_to_dict(request.target)
        speed = t_to_dict(request.speed)

        req = Request_Move_TickJogLT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed["spd_mode"]
        req.speed.spdVelPara = speed["spd_vel_para"]
        req.speed.spdAccPara = speed["spd_acc_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_tickjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_move_jb_clr(self, *, robot_model: str):
        req = Request_Move_JB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    def call_move_jb_add(self, *, robot_model: str, request: Request_MoveJBAddPD):
        target = t_to_dict(request.target)
        speed = t_to_dict(request.speed)
        type = t_to_dict(request.type)

        req = Request_Move_JB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed["spd_mode"]
        req.speed.spdVelPara = speed["spd_vel_para"]
        req.speed.spdAccPara = speed["spd_acc_para"]

        req.type.pntType = type["pnt_type"]
        req.type.pntPara = type["pnt_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_move_jb_run(self, *, robot_model: str):
        req = Request_Move_JB_RUNT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_jb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    def call_move_lb_clr(self, *, robot_model: str):
        req = Request_Move_LB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    def call_move_lb_add(self, *, robot_model: str, request: Request_MoveLBAddPD):
        target = t_to_dict(request.target)
        speed = t_to_dict(request.speed)
        type = t_to_dict(request.type)

        req = Request_Move_LB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed["spd_mode"]
        req.speed.spdVelPara = speed["spd_vel_para"]
        req.speed.spdAccPara = speed["spd_acc_para"]

        req.type.pntType = type["pnt_type"]
        req.type.pntPara = type["pnt_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_move_lb_run(self, *, robot_model: str, request: Request_MoveLBRunPD):
        orientation = t_to_dict(request.orientation)

        req = Request_Move_LB_RUNT()
        req.orientation = orientation

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    def call_move_xb_clr(self, *, robot_model: str):
        req = Request_Move_XB_CLRT()

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_clr",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]
    
    def call_move_xb_add(self, *, robot_model: str, request: Request_MoveXBAddPD):
        target = t_to_dict(request.target)
        speed = t_to_dict(request.speed)
        type = t_to_dict(request.type)

        req = Request_Move_XB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed["spd_mode"]
        req.speed.spdVelPara = speed["spd_vel_para"]
        req.speed.spdAccPara = speed["spd_acc_para"]

        req.type.pntType = type["pnt_type"]
        req.type.pntPara = type["pnt_para"]

        req.method = request.method

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]
    
    def call_move_xb_run(self, *, robot_model: str, request: Request_MoveXBRunPD):
        running_mode = t_to_dict(request.running_mode)

        req = Request_Move_XB_RUNT()
        req.runningMode = running_mode

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_xb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]
