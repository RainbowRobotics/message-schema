from rb_flat_buffers.SLAMNAV.MappingCloud import MappingCloudT
from rb_zenoh.router import ZenohRouter

from app.socket.socket_client import socket_client

map_zenoh_router = ZenohRouter()

@map_zenoh_router.subscribe(
    "amr/*/*/socket/mappingCloud",
    flatbuffer_obj_t=MappingCloudT
)
async def on_sub_slamnav_mappingCloud(*, topic, obj):
    # 로봇 모델과 아이디 추출
    robot_model, robot_id = topic.split("/")[1:3]
    # 로봇 모델과 아이디가 없으면 반환
    if not robot_model or not robot_id:
        return

    # 소켓 클라이언트로 전송
    await socket_client.emit(topic, obj)
