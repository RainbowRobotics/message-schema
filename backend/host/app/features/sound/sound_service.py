import platform
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
from rb_flat_buffers.IPC.Response_Sound_GetStatus import Response_Sound_GetStatusT
from rb_flat_buffers.IPC.Request_Sound_Play import Request_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Pause_Toggle import Response_Sound_Pause_ToggleT
from rb_flat_buffers.IPC.Response_Sound_Play import Response_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Stop import Response_Sound_StopT
from rb_modules.log import rb_log
from rb_utils.parser import t_to_dict
from rb_utils.service_exception import ServiceException
from .adapter.sound_linux_adapter import SoundLinuxAdapter
from .adapter.sound_windows_adapter import SoundWindowsAdapter
from .adapter.sound_mac_adapter import SoundMacAdapter
from .domain.sound import SoundModel

class SoundService:
    """
    [Sound 서비스]
    """
    def __init__(self):
        self.name = "sound"
        self.os = platform.system()
        if self.os.lower().__contains__("linux"):
            rb_log.info("[sound_service] Host OS : Linux")
            self.sound_port = SoundLinuxAdapter()
        elif self.os.lower().__contains__("windows"):
            self.sound_port = SoundWindowsAdapter()
            rb_log.info("[sound_service] Host OS : Windows")
        elif self.os.lower().__contains__("darwin"):
            rb_log.info("[sound_service] Host OS : Darwin")
            self.sound_port = SoundMacAdapter()
        else:
            rb_log.error(f"[sound_service] Unsupported OS: {self.os}")
            raise ValueError(f"Unsupported OS: {self.os}")

    async def get_sound(self) -> Response_Sound_GetStatusT:
        """
        [현재 사운드 재생 상태 조회]
        """

        return self.sound_port.get_status()

    async def play_sound(self, request: Request_Sound_PlayT) -> Response_Sound_PlayT:
        """
        [사운드 재생 요청]
        """
        try:
            dict_request = t_to_dict(request)
            rb_log.info(f"[sound_service] playSound request : {dict_request}")
            # 1) 모델 생성
            model = SoundModel()
            model.set_sound(request)

            # 2) 모델 검증
            model.check_variables()

            # 3) 사운드 재생
            return await self.sound_port.play(model)
        except ServiceException as e:
            rb_log.error(f"[sound_service] playSound ServiceException : {e.message} {e.status_code}")
            raise e

    async def stop_sound(self) -> Response_Sound_StopT:
        """
        [사운드 종료 요청]
        """
        try:
            rb_log.info("[sound_service] stopSound")

            # 1) 사운드 종료
            return await self.sound_port.stop()
        except ServiceException as e:
            rb_log.error(f"[sound_service] stopSound ServiceException : {e.message} {e.status_code}")
            raise e

    async def pause_toggle(self) -> Response_Sound_Pause_ToggleT:
        """
        [사운드 일시정지 토글 요청]
        """
        try:
            rb_log.info("[sound_service] pauseToggle")

            # 1) 사운드 일시정지 토글
            return await self.sound_port.pause()
        except ServiceException as e:
            rb_log.error(f"[sound_service] pauseToggle ServiceException : {e.message} {e.status_code}")
            raise e
