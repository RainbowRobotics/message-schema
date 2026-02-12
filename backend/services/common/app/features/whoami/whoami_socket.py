import asyncio
import time
from typing import Any

from rb_socketio import (
    RbSocketIORouter,
)

from app.socket.socket_client import (
    socket_client,
)

from .whoami_module import (
    WhoamiService,
)

whoami_socket_router = RbSocketIORouter()
whoamiService = WhoamiService()

_period = 0.1
_prev_res: list[dict[str, Any]] = []


@whoami_socket_router.on("{robot_model}/whoami")
async def on_whoami(data, robot_model: str):
    print("?????", robot_model, data)
    # next_ts = time.monotonic()

    # while True:
    #     res = await whoamiService.get_whoami(robot_model)

    #     if _prev_res != res:
    #         await socket_client.emit("whoami", res)

    #     next_ts += _period
    #     await asyncio.sleep(max(0, next_ts - time.monotonic()))
