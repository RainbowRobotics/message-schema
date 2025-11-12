from rb_database import get_db
from rb_flat_buffers.IPC.State_Log import (
    State_LogT,
)
from rb_zenoh.router import (
    ZenohRouter,
)
from rb_zenoh.schema import (
    SubscribeOptions,
)

from .log_module import (
    LogService,
)

zenoh_log_router = ZenohRouter()


log_service = LogService()


@zenoh_log_router.subscribe("*/state_log", flatbuffer_obj_t=State_LogT)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    db = await get_db()
    await log_service.on_zenoh_sub_state(db=db, topic=topic, obj=obj, attachment=attachment)


@zenoh_log_router.subscribe(
    "rrs_log", flatbuffer_obj_t=State_LogT, opts=SubscribeOptions(allowed_same_sender=True)
)
async def on_zenoh_sub_rrs_log(*, topic, mv, obj):
    db = await get_db()
    await log_service.on_zenoh_sub_rrs_log(db=db, obj=obj)
