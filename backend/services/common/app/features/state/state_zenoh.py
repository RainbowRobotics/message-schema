from flat_buffers.IPC.State_Core import State_CoreT
from flat_buffers.IPC.State_Message import State_MessageT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import OverflowPolicy, SubscribeOptions

from .state_module import StateService

state_service = StateService()
zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe(
    "*/state_core",
    flatbuffer_obj_t=State_CoreT,
    opts=SubscribeOptions(dispatch="queue", overflow=OverflowPolicy.LATEST_ONLY),
)
def on_manipulate_get_state(*, topic, mv, obj, attachment):
    state_service.get_zenoh_state_core(topic=topic, obj=obj)


@zenoh_state_router.subscribe("*/state_message", flatbuffer_obj_t=State_MessageT)
async def on_demo_state_message(*, topic, mv, obj, attachment):
    await state_service.get_state_message(topic=topic, message=obj)
