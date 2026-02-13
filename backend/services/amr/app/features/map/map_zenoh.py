from rb_flat_buffers.SLAMNAV.MappingCloud import MappingCloudT
from rb_zenoh.router import ZenohRouter

from app.socket.socket_client import socket_client

map_zenoh_router = ZenohRouter()

@map_zenoh_router.subscribe(
    "amr/{robot_model}/{robot_id}/socket/mappingCloud",
    flatbuffer_obj_t=MappingCloudT
)
async def on_sub_slamnav_mappingCloud(robot_model: str, robot_id: str, *, topic, obj):
    await socket_client.emit(topic, obj)
