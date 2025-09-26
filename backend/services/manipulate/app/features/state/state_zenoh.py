from app.socket.socket_client import socket_client
from flat_buffers.IPC.State_Core import State_CoreT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions
from utils.asyncio_helper import fire_and_log

zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe(
    "*/state_core",
    flatbuffer_obj_t=State_CoreT,
    opts=SubscribeOptions(dispatch="queue", overflow="latest_only"),
)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    fire_and_log(socket_client.emit(topic, obj))
