import asyncio
import time

from app.socket import socket_client
from flat_buffers.IPC.Request_CallWhoAmI import Request_CallWhoAmIT
from flat_buffers.IPC.Response_CallWhoamI import Response_CallWhoamIT
from rb_zenoh import zenoh_client


class WhoamiService:
    def __init__(self):
        self.period = 1
        self.prev_whoami_res = []

    async def get_whoami(self):
        req = Request_CallWhoAmIT()

        res = zenoh_client.query_all(
            "*/call_whoami",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallWhoamIT,
            flatbuffer_buf_size=1024,
        )

        return res

    async def repeat_get_whoami(self):
        next_ts = time.monotonic()

        while True:
            res = await self.get_whoami()

            if self.prev_whoami_res != res and socket_client.connected:
                await socket_client.emit("whoami", res)
                self.prev_whoami_res = res

            next_ts += self.period
            await asyncio.sleep(max(0, next_ts - time.monotonic()))
