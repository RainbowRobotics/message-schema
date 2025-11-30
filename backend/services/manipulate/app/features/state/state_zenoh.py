from app.socket.socket_client import socket_client
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_utils.asyncio_helper import fire_and_log
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import OverflowPolicy, SubscribeOptions

zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe(
    "*/state_core",
    flatbuffer_obj_t=State_CoreT,
    opts=SubscribeOptions(dispatch="queue", overflow=OverflowPolicy.LATEST_ONLY),
)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    fire_and_log(socket_client.emit(topic, obj))
