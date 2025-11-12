"""
[AMR 이동 Zenoh 어댑터]
"""
from app.features.move.src.application.amr_move_service import AmrMoveService
from rb_flat_buffers.SLAMNAV.State_Change_Move import State_Change_MoveT
from rb_zenoh.router import ZenohRouter
from rb_utils.parser import t_to_dict

move_zenoh_router = ZenohRouter()
amr_move_service = AmrMoveService()

@move_zenoh_router.subscribe(
    "*/v1/move",
    flatbuffer_obj_t=State_Change_MoveT
)
async def on_sub_slamnav_move_response(*, topic, obj):
    dict_obj = t_to_dict(obj)
    await amr_move_service.move_state_change(topic, dict_obj)
