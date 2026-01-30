from app.features.sound.port.sound_port import SoundPort
from app.features.sound.domain.sound import SoundModel

class SoundMacAdapter(SoundPort):
    """
    [Sound Mac 어댑터]
    """
    async def play_sound(self, model: SoundModel) -> dict:
        raise NotImplementedError("Not implemented")
    async def stop_sound(self) -> dict:
        raise NotImplementedError("Not implemented")
