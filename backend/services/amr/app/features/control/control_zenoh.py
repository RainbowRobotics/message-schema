"""
[AMR 이동 Zenoh 어댑터]
"""
from rb_flat_buffers.SLAMNAV.ResultControlDock import (
    ResultControlDockT,
)
from rb_zenoh.router import (
    ZenohRouter,
)
from rb_modules.log import rb_log
from .control_api import amr_control_service

control_zenoh_router = ZenohRouter()

@control_zenoh_router.subscribe(
    "*/control/result/dock",
    flatbuffer_obj_t=ResultControlDockT
)
async def on_sub_slamnav_control_dock_result(*, topic, obj):
    print("OnSub Slamnav Control Dock Result : ", obj)
    await amr_control_service.control_dock_result(topic, obj)
