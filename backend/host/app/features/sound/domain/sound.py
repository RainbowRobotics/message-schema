from dataclasses import dataclass, field
from enum import Enum
import os
import uuid

from rb_flat_buffers.IPC.Request_Sound_Play import Request_Sound_PlayT
from rb_utils.service_exception import ServiceException

class SoundStatusEnum(Enum):
    UNLOADED = "unloaded"
    LOADED = "loaded"
    PLAYING = "playing"
    STOPPED = "stopped"
    PAUSED = "paused"
    RESUMED = "resumed"


@dataclass()
class SoundModel:
    """
    [Sound 모델]
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    status: SoundStatusEnum = SoundStatusEnum.UNLOADED
    file_name: str | None = None
    file_path: str | None = None
    volume: int | None = None
    repeat_count: int | None = None


    def set_sound(self, request: Request_Sound_PlayT):
        self.status = SoundStatusEnum.LOADED
        self.file_name = request.fileName
        self.file_path = request.filePath
        self.volume = request.volume
        self.repeat_count = request.repeatCount

    def check_variables(self) -> None:
        """
        [요청 변수 검사]
        """
        if self.file_name is None and self.file_path is None:
            raise ServiceException("file_name 또는 file_path 값이 없습니다.", 403)
        if self.file_name is not None and self.file_path is None:
            self.file_path = f"./public/sounds/{self.file_name}"
        if not os.path.exists(self.file_path):
            raise ServiceException(f"파일이 존재하지 않습니다. ({self.file_path})", 404)
        if self.volume is None or self.volume < 0:
            raise ServiceException("volume 값이 없거나 0보다 작습니다.", 403)
        if self.repeat_count is None or self.repeat_count < 0:
            self.repeat_count = 1
