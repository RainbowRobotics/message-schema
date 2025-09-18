import flatbuffers
from flat_buffers.IPC.Request_State_Whoami import Request_State_WhoamiT
from flat_buffers.IPC.State_Whoami import State_WhoamiT
from rb_zenoh import zenoh_client
from utils.parser import t_to_dict


class WhoamiService:
    def __init__(self):
        pass

    async def get_whoami(self):
        req = Request_State_WhoamiT()

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one("*/call_whoami", payload=fb_payload)

        buf = res["payload"]
        res = State_WhoamiT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)
