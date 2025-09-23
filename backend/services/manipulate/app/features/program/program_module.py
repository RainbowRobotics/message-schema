from flat_buffers.IPC.N_CARTE_f import N_CARTE_fT
from flat_buffers.IPC.N_JOINT_f import N_JOINT_fT
from flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from flat_buffers.IPC.Request_MotionSmoothJogJ import Request_MotionSmoothJogJT
from flat_buffers.IPC.Request_MotionSmoothJogL import Request_MotionSmoothJogLT
from flat_buffers.IPC.Request_MotionSmoothJogStop import Request_MotionSmoothJogStopT
from flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from flat_buffers.IPC.Request_Move_J import Request_Move_JT
from flat_buffers.IPC.Request_Move_L import Request_Move_LT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh import zenoh_client


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

    async def call_smoothjogj(
        self, *, robot_model: str, targetspeed: list[float], frame: int, unit: int
    ):
        req = Request_MotionSmoothJogJT()

        nj = N_JOINT_fT()
        nj.f = targetspeed
        req.targetspeed = nj

        req.frame = frame
        req.unit = unit

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
        req = Request_MotionSmoothJogLT()

        ncf = N_CARTE_fT()
        ncf.f = targetspeed
        req.targetspeed = ncf

        req.frame = frame
        req.unit = unit

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_smoothjog_stop(self, *, robot_model: str, stoptime: float):
        req = Request_MotionSmoothJogStopT()
        req.stoptime = stoptime

        res = zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return res["dict_payload"]

    async def call_move_j(
        self,
        *,
        robot_model: str,
        targetspeed: list[float],
        frame: int,
        unit: int,
        speed_rate: float,
        accel_rate: float,
    ):
        req = Request_Move_JT()
        nj = N_JOINT_fT()
        nj.f = targetspeed
        req.targetspeed = nj
        req.frame = frame
        req.unit = unit
        req.speed_rate = speed_rate
        req.accel_rate = accel_rate
        res = zenoh_client.query_one(
            f"{robot_model}/call_move_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]

    async def call_move_l(
        self,
        *,
        robot_model: str,
        targetspeed: list[float],
        frame: int,
        unit: int,
        speed_mmps: float,
        accel_mmpss: float,
    ):
        req = Request_Move_LT()
        ncf = N_CARTE_fT()
        ncf.f = targetspeed
        req.targetspeed = ncf
        req.frame = frame
        req.unit = unit
        req.speed_mmps = speed_mmps
        req.accel_mmpss = accel_mmpss
        res = zenoh_client.query_one(
            f"{robot_model}/call_move_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        return res["dict_payload"]
