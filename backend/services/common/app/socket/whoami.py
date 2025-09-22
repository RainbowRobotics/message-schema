import asyncio
import time

from rb_socketio import RbSocketIORouter

from app.modules.whomai.whoami_module_service import WhoamiService
from app.socket import sio

whoami_socket_router = RbSocketIORouter()
whoamiService = WhoamiService()

_period = 0.1
_prev_res = []


@whoami_socket_router.on("whoami")
async def on_whoami(data):
    next_ts = time.monotonic()

    while True:
        res = await whoamiService.get_whoami()

        if _prev_res != res:
            await sio.emit("whoami", res)

        next_ts += _period
        await asyncio.sleep(max(0, next_ts - time.monotonic()))
