"""
[Email API]
"""



from fastapi import APIRouter
from rb_modules.log import RBLog

from app.features.sound.sound_schema import (
    Request_Sound_PlayPD,
)

from .sound_module import (
    SoundService,
)

sound_service = SoundService()
sound_router = APIRouter(
    tags=["Sound"],
    prefix="/sound"
)
rb_log = RBLog()

@sound_router.get(
    "/status",
    summary="현재 사운드 재생 상태 조회",
    description="""
현재 사운드 재생 상태를 조회합니다.

## 📌 응답 바디(JSON)

| 필드명 | 타입 | 설명 | 예시 |
|-|-|-|-|
| fileName | string | 사운드 파일 이름 |  |
| status | string | 사운드 상태 | loaded, playing, stopped, paused, resumed |
| volume | int | 사운드 볼륨 | 0 ~ 100 |
| repeatCount | int | 사운드 반복 횟수 | 1 |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
    """
)
async def get_status():
    """
    [현재 네트워크 조회(이더넷,와이파이,블루투스)]
    """
    return await sound_service.get_status()

@sound_router.post(
    "/play",
    summary="사운드 재생",
    description="""
사운드를 재생합니다.
    """
)
async def play_sound(request: Request_Sound_PlayPD):
    return await sound_service.play_sound(request)

@sound_router.post(
    "/stop",
    summary="사운드 종료",
    description="""
사운드를 종료합니다.
    """
)
async def stop_sound():
    return await sound_service.stop_sound()


@sound_router.post(
  "/pause",
    summary="사운드 일시정지 토글",
    description="""
사운드를 일시정지합니다. 이미 일시정지인 상태라면 해제합니다.
    """
)
async def pause_sound():
    return await sound_service.pause_toggle()
