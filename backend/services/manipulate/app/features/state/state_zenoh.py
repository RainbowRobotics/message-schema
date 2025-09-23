from app.socket.socket_client import socket_client
from fastapi import HTTPException
from flat_buffers.IPC.State_Core import State_CoreT
from rb_zenoh.router import ZenohRouter

zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe("*/state_core", flatbuffer_obj_t=State_CoreT)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    try:
        await socket_client.emit(topic, obj)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(f"error: {e}")) from e
