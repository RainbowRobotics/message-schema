
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.GlobalPath import GlobalPathT
from rb_flat_buffers.SLAMNAV.LocalPath import LocalPathT
from rb_flat_buffers.SLAMNAV.MappingCloud import MappingCloudT
from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2D
from rb_socketio import RBSocketIONsClient
from rb_zenoh.client import ZenohClient
from rb_zenoh.router import (
    ZenohRouter,
)
from rb_zenoh.schema import (
    SubscribeOptions,
)

amr_zenoh_router = ZenohRouter()
zenoh_client = ZenohClient()

socket_client = RBSocketIONsClient(
    "amr",
    reconnection=True,  # 자동 재연결 활성화
    reconnection_attempts=0,  # 0 = 무제한
    reconnection_delay=1,  # 최초 재시도 간격(초)
    reconnection_delay_max=30,  # 백오프 상한
)

@amr_zenoh_router.subscribe(
    "*/status",
    flatbuffer_obj_t=StatusT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only"),
)
async def on_sub_slamnav_status(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/moveStatus",
    flatbuffer_obj_t=MoveStatusT
)
async def on_sub_slamnav_moveStatus(*, topic, obj):
    # print("================MOVESTATUS=================", flush=True)
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/socket/globalPath",
    flatbuffer_obj_t=GlobalPathT
)
async def on_sub_slamnav_globalPath(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/socket/localPath",
    flatbuffer_obj_t=LocalPathT
)
async def on_sub_slamnav_localPath(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/socket/mappingCloud",
    flatbuffer_obj_t=MappingCloudT
)
async def on_sub_slamnav_mappingCloud(*, topic, obj):
    await socket_client.emit(topic, obj)

@amr_zenoh_router.subscribe(
    "*/socket/lidar2d",
    flatbuffer_obj_t=Lidar2D
)
async def on_sub_slamnav_lidar2d(*, topic, obj):
    await socket_client.emit(topic, obj)
