from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from app.socket.socket_client import socket_client

status_zenoh_router = ZenohRouter()

@status_zenoh_router.subscribe(
    "amr/{robot_model}/{robot_id}/status",
    flatbuffer_obj_t=StatusT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_status(robot_model: str, robot_id: str, *, topic, obj):
    print("Status Sub : ", robot_model, robot_id, topic, obj, flush=True)
    await socket_client.emit(topic, obj)

@status_zenoh_router.subscribe(
    "amr/{robot_model}/{robot_id}/moveStatus",
    flatbuffer_obj_t=MoveStatusT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_moveStatus(robot_model: str, robot_id: str, *, topic, obj):
    await socket_client.emit(topic, obj)

@status_zenoh_router.subscribe(
    "amr/{robot_model}/{robot_id}/lidar2d",
    flatbuffer_obj_t=Lidar2DT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_lidar2d(robot_model: str, robot_id: str, *, topic, obj):
    await socket_client.emit(topic, obj)
