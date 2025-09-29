from rb_socketio import RbSocketIORouter
from utils.parser import to_json

from .program_module import ProgramService

program_socket_router = RbSocketIORouter()
program_service = ProgramService()


@program_socket_router.on("speedbar")
async def on_change_speedbar(data):
    res = await program_service.control_speed_bar(
        components=data["components"], speedbar=data["speedbar"]
    )

    return to_json(res)
