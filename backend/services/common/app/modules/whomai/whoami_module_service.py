import flatbuffers
from flat_buffers.IPC.Request_CallWhoAmI import Request_CallWhoAmIT
from flat_buffers.IPC.Response_CallWhoamI import Response_CallWhoamIT
from rb_zenoh import zenoh_client
from utils.parser import t_to_dict


class WhoamiService:
    def __init__(self):
        pass

    async def get_whoami(self):
        req = Request_CallWhoAmIT()

        b = flatbuffers.Builder(32)
        b.Finish(req.Pack(b))
        fb_payload = bytes(b.Output())

        res = zenoh_client.query_one("*/call_whoami", payload=fb_payload)

        buf = res["payload"]
        res = Response_CallWhoamIT.InitFromPackedBuf(buf, 0)

        return t_to_dict(res)
