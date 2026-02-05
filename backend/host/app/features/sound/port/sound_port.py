from rb_flat_buffers.IPC.Response_Sound_GetStatus import Response_Sound_GetStatusT
from app.features.sound.domain.sound import SoundModel

from typing import(
    Any,
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.IPC.Response_Sound_Play import Response_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Stop import Response_Sound_StopT
from rb_flat_buffers.IPC.Response_Sound_Pause_Toggle import Response_Sound_Pause_ToggleT

@runtime_checkable
class SoundPort(Protocol):
    """
    [Sound 포트]
    """
    def get_status(self) -> Response_Sound_GetStatusT: ...
    async def play(self, model: SoundModel) -> Response_Sound_PlayT: ...
    async def stop(self) -> Response_Sound_StopT: ...
    async def pause(self) -> Response_Sound_Pause_ToggleT: ...
