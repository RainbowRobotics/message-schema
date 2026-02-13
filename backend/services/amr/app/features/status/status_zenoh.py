from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from app.features.status.status_api import amr_status_service

status_zenoh_router = ZenohRouter()

@status_zenoh_router.subscribe(
    "amr/*/*/status",
    flatbuffer_obj_t=StatusT,
    opts=SubscribeOptions(dispatch="queue", maxsize=2, overflow="latest_only")
)
async def on_sub_slamnav_status(*, topic, obj):
    await amr_status_service.set_status(topic, obj)

@status_zenoh_router.subscribe(
    "amr/*/*/moveStatus",
    flatbuffer_obj_t=MoveStatusT,
    opts=SubscribeOptions(dispatch="queue", maxsize=2, overflow="latest_only")
)
async def on_sub_slamnav_moveStatus(*, topic, obj):
    await amr_status_service.set_move_status(topic, obj)

@status_zenoh_router.subscribe(
    "amr/*/*/socket/lidar2d",
    flatbuffer_obj_t=Lidar2DT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_lidar2d(*, topic, obj):
    await amr_status_service.set_lidar2d(topic, obj)
