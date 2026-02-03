"""
[AMR 이동 Zenoh 어댑터]
"""
from rb_flat_buffers.SLAMNAV.ResultMove import ResultMoveT
from rb_zenoh.router import (
    ZenohRouter,
)
from .move_api import amr_move_service
move_zenoh_router = ZenohRouter()

@move_zenoh_router.subscribe(
    "*/move/result",
    flatbuffer_obj_t=ResultMoveT
)
async def on_sub_slamnav_move_result(*, topic, obj):
    print("OnSub Slamnav Move Result : ", obj)
    await amr_move_service.move_result(topic, obj)
