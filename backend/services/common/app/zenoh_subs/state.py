from fastapi import HTTPException
from flat_buffers.IPC.State_Message import State_MessageT
from rb_zenoh.router import ZenohRouter

from app.socket import socket_client

zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe("*/state_message", flatbuffer_obj_t=State_MessageT)
async def on_demo_state_message(*, topic, mv, obj, attachment):
    try:
        sw_name = topic.split("/")[0]

        obj["swName"] = sw_name
        await socket_client.emit("state_message", obj)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(f"error: {e}")) from e
