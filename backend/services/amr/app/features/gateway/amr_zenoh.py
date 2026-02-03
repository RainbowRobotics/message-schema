
from rb_flat_buffers.SLAMNAV.Status import (
    StatusT,
)
from rb_zenoh.client import ZenohClient
from rb_zenoh.router import (
    ZenohRouter,
)
from rb_zenoh.schema import (
    SubscribeOptions,
)

from app.socket.socket_client import (
    socket_client,
)

amr_zenoh_router = ZenohRouter()
zenoh_client = ZenohClient()

@amr_zenoh_router.subscribe(
    "*/status",
    flatbuffer_obj_t=StatusT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only"),
)
async def on_sub_slamnav_status(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/globalPath",
    flatbuffer_obj_t=PathT
)
async def on_sub_slamnav_globalPath(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/localPath",
    flatbuffer_obj_t=PathT
)
async def on_sub_slamnav_localPath(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/moveStatus",
    flatbuffer_obj_t=MoveStatusT
)
async def on_sub_slamnav_moveStatus(*, topic, obj):
    # print("================MOVESTATUS=================", flush=True)
    await socket_client.emit(topic, obj)
