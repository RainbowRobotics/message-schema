from fastapi import HTTPException
from flat_buffers.IPC.State_Core import State_CoreT
from rb_zenoh.router import ZenohRouter
from utils.parser import t_to_dict

from app.socket import socket_client

zenoh_state_router = ZenohRouter()


@zenoh_state_router.subscribe("cobot/state_core")
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    try:
        buf = bytes(mv)
        obj = State_CoreT.InitFromPackedBuf(buf, 0)

        await socket_client.emit(topic, t_to_dict(obj))
    except Exception as e:
        raise HTTPException(
            status_code=502, detail=str(f"flatbuffers decode error Tlqkf: {e}")
        ) from e
