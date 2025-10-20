import rb_database.mongo_db as mongo_db
from rb_socketio import RbSocketIORouter
from utils.parser import to_json

from .info_module import InfoService

info_service = InfoService()
info_socket_router = RbSocketIORouter()


@info_socket_router.on("robot-info")
async def on_robot_info(_):
    res = await info_service.get_robot_info(db=mongo_db.db)
    return to_json(res)


@info_socket_router.on("{robot_model}/robot-urdf-link-map")
async def on_robot_urdf_link_map(_, robot_model: str):
    res = await info_service.get_robot_urdf_link_map(robot_model=robot_model)
    return to_json(res)
