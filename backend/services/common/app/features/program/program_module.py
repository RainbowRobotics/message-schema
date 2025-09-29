import flatbuffers
from fastapi import HTTPException
from flat_buffers.IPC.Request_MotionSmoothJogStop import Request_MotionSmoothJogStopT
from flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.parser import t_to_dict

zenoh_client = ZenohClient()


class ProgramService:
    def __init__(self) -> None:
        pass

    async def control_speed_bar(self, *, speedbar: int):
        try:
            req = Request_MotionSpeedBarT()
            req.alpha = speedbar

            b = flatbuffers.Builder(32)
            b.Finish(req.Pack(b))
            fb_payload = bytes(b.Output())

            res = zenoh_client.query_one("*/call_speedbar", payload=fb_payload)

            buf = res["payload"]
            res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

    async def call_smoothjog_stop(self, *, stoptime: float):
        req = Request_MotionSmoothJogStopT()
        req.stoptime = stoptime

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one("*/call_smoothjog_stop", payload=fb_payload)

        buf = res["payload"]
        res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)
