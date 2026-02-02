from rb_flat_buffers.IPC.Response_Sound_GetStatus import Response_Sound_GetStatusT
from rb_flat_buffers.IPC.Request_Sound_Play import Request_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Pause_Toggle import Response_Sound_Pause_ToggleT
from rb_flat_buffers.IPC.Response_Sound_Play import Response_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Stop import Response_Sound_StopT
from rb_zenoh.router import ZenohRouter
from .sound_service import SoundService

sound_zenoh_router = ZenohRouter()
sound_service = SoundService()

@sound_zenoh_router.queryable("sound/status", flatbuffer_res_buf_size=2048)
async def on_query_sound() -> Response_Sound_GetStatusT:
    return await sound_service.get_sound()


@sound_zenoh_router.queryable("sound/play", flatbuffer_req_t=Request_Sound_PlayT, flatbuffer_res_buf_size=2048)
async def on_query_sound_play(req: Request_Sound_PlayT) -> Response_Sound_PlayT:
    return await sound_service.play_sound(req)

@sound_zenoh_router.queryable("sound/stop", flatbuffer_res_buf_size=2048)
async def on_query_sound_stop() -> Response_Sound_StopT:
    return await sound_service.stop_sound()

@sound_zenoh_router.queryable("sound/pause", flatbuffer_res_buf_size=2048)
async def on_query_sound_pause() -> Response_Sound_Pause_ToggleT:
    return await sound_service.pause_toggle()
