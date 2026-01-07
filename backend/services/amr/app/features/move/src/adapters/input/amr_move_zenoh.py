"""
[AMR 이동 Zenoh 어댑터]
"""
from rb_flat_buffers.SLAMNAV.State_Change_Move import (
    State_Change_MoveT,
)
from rb_zenoh.router import (
    ZenohRouter,
)

from .amr_move_api import amr_move_service

move_zenoh_router = ZenohRouter()

@move_zenoh_router.subscribe(
    "*/v1/move",
    flatbuffer_obj_t=State_Change_MoveT
)
async def on_sub_slamnav_move_response(*, topic, obj):
    print("OnSub Slamnav Move Response : ", obj)
    await amr_move_service.move_state_change(topic, obj)
