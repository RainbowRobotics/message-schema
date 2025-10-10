from app.features.program.program_schema import MoveInputSpeed, MoveInputTarget
from flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from flat_buffers.IPC.MoveInput_Type import MoveInput_TypeT
from flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from flat_buffers.IPC.Request_Move_J import Request_Move_JT
from flat_buffers.IPC.Request_Move_JB_ADD import Request_Move_JB_ADDT
from flat_buffers.IPC.Request_Move_JB_CLR import Request_Move_JB_CLRT
from flat_buffers.IPC.Request_Move_JB_RUN import Request_Move_JB_RUNT
from flat_buffers.IPC.Request_Move_L import Request_Move_LT
from flat_buffers.IPC.Request_Move_LB_ADD import Request_Move_LB_ADDT
from flat_buffers.IPC.Request_Move_LB_CLR import Request_Move_LB_CLRT
from flat_buffers.IPC.Request_Move_LB_RUN import Request_Move_LB_RUNT
from flat_buffers.IPC.Request_Move_SmoothJogJ import Request_Move_SmoothJogJT
from flat_buffers.IPC.Request_Move_SmoothJogL import Request_Move_SmoothJogLT
from flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from flat_buffers.IPC.Request_Move_TickJogJ import Request_Move_TickJogJT
from flat_buffers.IPC.Request_Move_TickJogL import Request_Move_TickJogLT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh.client import ZenohClient

from .program_schema import (
    Request_MoveJBAddPD,
    Request_MoveLBAddPD,
    Request_MoveLBRunPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
)

zenoh_client = ZenohClient()


class ProgramService:
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

        nj = N_JOINT_fT()
        nj.f = targetspeed
        req.target.tarValues = nj

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

        nf = N_JOINT_fT()
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
        req = Request_Move_JT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        move_input_target.tarValues = ni
        move_input_target.tarFrame = target["tar_frame"]
        move_input_target.tarUnit = target["tar_unit"]

        move_input_speed.spdMode = speed["spd_mode"]
        move_input_speed.spdVelPara = speed["spd_vel_para"]
        move_input_speed.spdAccPara = speed["spd_acc_para"]

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
        req = Request_Move_LT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        move_input_target.tarValues = ni
        move_input_target.tarFrame = target["tar_frame"]
        move_input_target.tarUnit = target["tar_unit"]

        move_input_speed.spdMode = speed["spd_mode"]
        move_input_speed.spdVelPara = speed["spd_vel_para"]
        move_input_speed.spdAccPara = speed["spd_acc_para"]

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
        req = Request_Move_TickJogJT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = request.target["tar_frame"]
        req.target.tarUnit = request.target["tar_unit"]

        req.speed.spdMode = request.speed["spd_mode"]
        req.speed.spdVelPara = request.speed["spd_vel_para"]
        req.speed.spdAccPara = request.speed["spd_acc_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_tickjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_tickjog_l(self, *, robot_model: str, request: Request_MoveTickJogLPD):
        req = Request_Move_TickJogLT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = request.target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = request.target["tar_frame"]
        req.target.tarUnit = request.target["tar_unit"]

        req.speed.spdMode = request.speed["spd_mode"]
        req.speed.spdVelPara = request.speed["spd_vel_para"]
        req.speed.spdAccPara = request.speed["spd_acc_para"]

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
        req = Request_Move_JB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = request.target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = request.target["tar_frame"]
        req.target.tarUnit = request.target["tar_unit"]

        req.speed.spdMode = request.speed["spd_mode"]
        req.speed.spdVelPara = request.speed["spd_vel_para"]
        req.speed.spdAccPara = request.speed["spd_acc_para"]

        req.type.pntType = request.type["pnt_type"]
        req.type.pntPara = request.type["pnt_para"]

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
        req = Request_Move_LB_ADDT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()
        req.type = MoveInput_TypeT()

        ni = N_INPUT_fT()
        ni.f = request.target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = request.target["tar_frame"]
        req.target.tarUnit = request.target["tar_unit"]

        req.speed.spdMode = request.speed["spd_mode"]
        req.speed.spdVelPara = request.speed["spd_vel_para"]
        req.speed.spdAccPara = request.speed["spd_acc_para"]

        req.type.pntType = request.type["pnt_type"]
        req.type.pntPara = request.type["pnt_para"]

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_add",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    def call_move_lb_run(self, *, robot_model: str, request: Request_MoveLBRunPD):
        req = Request_Move_LB_RUNT()
        req.orientation = request.orientation

        res = zenoh_client.query_one(
            f"{robot_model}/call_move_lb_run",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]
