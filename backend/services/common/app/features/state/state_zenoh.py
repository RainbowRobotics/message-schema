from flat_buffers.IPC.State_Message import State_MessageT
from rb_zenoh.router import ZenohRouter

from .state_module import StateService

state_service = StateService()
zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe("*/state_message", flatbuffer_obj_t=State_MessageT)
async def on_demo_state_message(*, topic, mv, obj, attachment):
    await state_service.get_state_message(topic=topic, message=obj)
