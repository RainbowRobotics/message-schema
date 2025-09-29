import rb_database.mongo_db as mongo_db
from app.features.log.log_module import LogService
from flat_buffers.IPC.State_Log import State_LogT
from rb_zenoh import SubscribeOptions
from rb_zenoh.router import ZenohRouter

zenoh_log_router = ZenohRouter()


log_service = LogService()


@zenoh_log_router.subscribe("*/state_log", flatbuffer_obj_t=State_LogT)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    await log_service.on_zenoh_sub_state(
        db=mongo_db.db, topic=topic, obj=obj, attachment=attachment
    )


@zenoh_log_router.subscribe(
    "rrs_log", flatbuffer_obj_t=State_LogT, opts=SubscribeOptions(allowed_same_sender=True)
)
async def on_zenoh_sub_rrs_log(*, topic, mv, obj):
    await log_service.on_zenoh_sub_rrs_log(db=mongo_db.db, obj=obj)
