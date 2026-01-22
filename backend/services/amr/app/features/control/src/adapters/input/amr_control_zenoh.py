# """
# [AMR 이동 Zenoh 어댑터]
# """
# from rb_flat_buffers.SLAMNAV.State_Change_Move import (
#     State_Change_MoveT,
# )
# from rb_zenoh.router import (
#     ZenohRouter,
# )

# from .amr_control_api import amr_control_service

# control_zenoh_router = ZenohRouter()

# @control_zenoh_router.subscribe(
#     "*/v1/control",
#     flatbuffer_obj_t=State_Change_ControlT
# )
# async def on_sub_slamnav_control_response(*, topic, obj):
#     print("OnSub Slamnav Control Response : ", obj)
#     await amr_control_service.control_state_change(topic, obj)
