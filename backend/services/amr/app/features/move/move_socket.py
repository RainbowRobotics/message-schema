from rb_socketio import RbSocketIORouter

from app.features.move.move_api import amr_move_service
from app.features.move.move_schema import RequestMoveJogPD

move_socket_router = RbSocketIORouter()

@move_socket_router.on("{robot_model}/{robot_id}/move/jog")
async def on_sub_slamnav_move_jog(data, robot_model: str, robot_id: str):
    """
    Socket -> Zenoh (MoveJog) 요청 처리
    """
    request = RequestMoveJogPD(vx=data.get("vx", 0.0), vy=data.get("vy", 0.0), wz=data.get("wz", 0.0))
    await amr_move_service.move_jog(robot_model, robot_id, request=request)
