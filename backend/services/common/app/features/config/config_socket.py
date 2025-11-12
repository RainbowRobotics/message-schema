from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    to_json,
)

from .config_module import (
    ConfigService,
)

config_socket_router = RbSocketIORouter()
config_service = ConfigService()


@config_socket_router.on("speedbar")
async def on_change_speedbar(data):
    if data.get("components") is None:
        return

    res = await config_service.control_speed_bar(
        components=data["components"], speedbar=data["speedbar"]
    )

    return to_json(res)
