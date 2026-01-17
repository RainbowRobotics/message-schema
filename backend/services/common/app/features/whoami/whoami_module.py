from rb_flat_buffers.IPC.Request_CallWhoAmI import (
    Request_CallWhoAmIT,
)
from rb_flat_buffers.IPC.Response_CallWhoamI import (
    Response_CallWhoamIT,
)
from rb_zenoh.client import (
    ZenohClient,
)
from rb_zenoh.exeption import ZenohNoReply

zenoh_client = ZenohClient()


class WhoamiService:
    def __init__(self):
        self.period = 1
        self.prev_whoami_res = []

    async def get_whoami(self, robot_model: str):
        req = Request_CallWhoAmIT()

        try:

            res = zenoh_client.query_one(
                f"{robot_model}/call_whoami",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_CallWhoamIT,
                flatbuffer_buf_size=8,
            )
        except ZenohNoReply:
            return {}

        return res
