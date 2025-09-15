import flatbuffers
from flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from flat_buffers.IPC.Request_MotionSmoothJogJ import Request_MotionSmoothJogJT
from flat_buffers.IPC.Request_MotionSmoothJogL import Request_MotionSmoothJogLT
from flat_buffers.IPC.Request_MotionSmoothJogStop import Request_MotionSmoothJogStopT
from flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh import zenoh_client
from utils.parser import t_to_dict


class ProgramService:
    def __init__(self):
        pass

    def call_resume(self, *, robot_model: str):
        req = Request_MotionPauseT()

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_resume", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)

    def call_pause(self, *, robot_model: str):
        req = Request_MotionPauseT()

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_pause", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)

    def call_speedbar(self, *, robot_model: str, speedbar: float):
        req = Request_MotionSpeedBarT()
        req.alpha = speedbar

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_speedbar", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)

    def call_smoothjogj(self, *, robot_model: str, targetspeed: list[float], frame: int, unit: int):
        req = Request_MotionSmoothJogJT()
        req.targetspeed = targetspeed
        req.frame = frame
        req.unit = unit

        b = flatbuffers.Builder(256)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_smoothjog_j", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)

    def call_smoothjogl(self, *, robot_model: str, targetspeed: list[float], frame: int, unit: int):
        req = Request_MotionSmoothJogLT()
        req.targetspeed = targetspeed
        req.frame = frame
        req.unit = unit

        b = flatbuffers.Builder(256)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_smoothjog_l", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)

    def call_smoothjogstop(self, *, robot_model: str, stoptime: float):
        req = Request_MotionSmoothJogStopT()
        req.stoptime = stoptime

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one(f"{robot_model}/call_smoothjog_stop", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)
