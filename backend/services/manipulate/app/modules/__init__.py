import flatbuffers
from fastapi import HTTPException
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh import zenoh_client
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.parser import t_to_dict


class BaseZenohService:
    async def _send_query(self, topic: str, req_obj, bufsize: int = 32) -> dict:
        """공통 Zenoh query 처리"""
        try:
            b = flatbuffers.Builder(bufsize)
            b.Finish(req_obj.Pack(b))
            fb_payload = bytes(b.Output())

            res = zenoh_client.query_one(topic, payload=fb_payload)
            buf = res["payload"]

            res = Response_FunctionsT.InitFromPackedBuf(buf, 0)
            return t_to_dict(res)

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=f"error: {e}") from e
