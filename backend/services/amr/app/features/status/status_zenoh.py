from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from app.socket.socket_client import socket_client

status_zenoh_router = ZenohRouter()

@status_zenoh_router.subscribe(
    "amr/*/*/status",
    flatbuffer_obj_t=StatusT,
    opts=SubscribeOptions(dispatch="queue", maxsize=2, overflow="latest_only")
)
async def on_sub_slamnav_status(*, topic, obj):
    # 로봇 모델과 아이디 추출
    robot_model, robot_id = topic.split("/")[1:3]
    # 로봇 모델과 아이디가 없으면 반환
    if not robot_model or not robot_id:
        return
    # print("Status Sub : ", robot_model, robot_id, topic, obj, flush=True)
    # 소켓 클라이언트로 전송
    await socket_client.emit(topic, obj)

    # common으로 전송
    # await status_zenoh_router.publish(f"?/status", obj)


@status_zenoh_router.subscribe(
    "amr/*/*/moveStatus",
    flatbuffer_obj_t=MoveStatusT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_moveStatus(*, topic, obj):
    # 로봇 모델과 아이디 추출
    robot_model, robot_id = topic.split("/")[1:3]

    # 로봇 모델과 아이디가 없으면 반환
    if not robot_model or not robot_id:
        return

    # print("MoveStatus Sub : ", robot_model, robot_id, topic, obj, flush=True)

    # 소켓 클라이언트로 전송
    await socket_client.emit(topic, obj)

    # common으로 전송
    # await status_zenoh_router.publish(f"?/moveStatus", obj)

@status_zenoh_router.subscribe(
    "amr/*/*/socket/lidar2d",
    flatbuffer_obj_t=Lidar2DT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only")
)
async def on_sub_slamnav_lidar2d(*, topic, obj):
    # 로봇 모델과 아이디 추출
    robot_model, robot_id = topic.split("/")[1:3]

    # 로봇 모델과 아이디가 없으면 반환
    if not robot_model or not robot_id:
        return

    # 소켓 클라이언트로 전송
    await socket_client.emit(topic, obj)

    # common으로 전송
    # await status_zenoh_router.publish(f"?/lidar2d", obj)
