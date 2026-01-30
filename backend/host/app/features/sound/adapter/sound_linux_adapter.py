import asyncio
import subprocess
import time
import threading
from rb_flat_buffers.IPC.Response_Sound_GetStatus import Response_Sound_GetStatusT
from rb_flat_buffers.IPC.Response_Sound_Pause_Toggle import Response_Sound_Pause_ToggleT
from rb_flat_buffers.IPC.Response_Sound_Play import Response_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Stop import Response_Sound_StopT
from rb_modules.log import rb_log
from rb_utils.service_exception import ServiceException
from app.features.sound.port.sound_port import SoundPort
from app.features.sound.domain.sound import SoundModel, SoundStatusEnum

class SoundLinuxAdapter(SoundPort):
    """
    [Sound Linux 어댑터]
    """

    def __init__(self):
        # 현재 재생 중인 모델 정보
        self.current_sound = None
        # mplayer 프로세스
        self.proc = None
        # 일시정지 여부
        self._paused = False
        # 락
        self._lock = asyncio.Lock()
        # 감시 스레드
        self._watch_thread = None
        # 감시 스레드 중지 이벤트
        self._watch_stop = threading.Event()


    def get_status(self) -> Response_Sound_GetStatusT:
        """
        [현재 사운드 재생 상태 조회]
        """
        if self.current_sound is None:
            return Response_Sound_GetStatusT(
                fileName="",
                volume=0,
                repeatCount=0,
                status="NONE"
            )
        else:
            return Response_Sound_GetStatusT(
                fileName=self.current_sound.file_name,
                volume=self.current_sound.volume,
                repeatCount=self.current_sound.repeat_count,
                status=self.current_sound.status.value
            )

    def _watch_loop(self):
        """
        [mplayer 프로세스 감시 루프]
        - play() 에서 실행됨
        - watch_stop 이벤트가 설정되면 종료
        - proc.poll()은 실행중이면 None을 반환, 종료된 상태면 종료코드(int) 반환
        """
        while not self._watch_stop.is_set():
            p = self.proc
            if p is not None:
                if p.poll() is not None:
                    rb_log.debug(f"[sound_linux_adapter] 플레이 종료 감지 : {self.current_sound.file_name if self.current_sound is not None else 'None'}")
                    # 현재 proc가 p일 때만 정리
                    if self.proc is p:
                        self.proc = None
                        self.current_sound = None
                        self._paused = False
                        self._watch_stop.set()
                else:
                    # 프로세스가 실행중이면 상태를 PLAYING으로 변경
                    if self.current_sound is not None:
                        if self._paused:
                            self.current_sound.status = SoundStatusEnum.PAUSED
                        else:
                            self.current_sound.status = SoundStatusEnum.PLAYING
            time.sleep(0.1)


    async def play(self, model: SoundModel) -> Response_Sound_PlayT:
        """
        [사운드 재생]
        - 현재 재생 중이면 종료 후 재생
        - 현재 재생 중이 아니면 재생
        """
        async with self._lock:
            # 1) 현재 재생 중이면 종료
            if self.proc is not None:
                await self.stop()

            # 2) 변수 세팅
            self.current_sound = model
            self._paused = False

            # 3) watch 스레드 시작
            if self._watch_thread is None or not self._watch_thread.is_alive():
                self._watch_stop.clear()
                self._watch_thread = threading.Thread(target=self._watch_loop, daemon=True)
                self._watch_thread.start()

            # 4) mplayer 프로세스 실행
            self.proc = subprocess.Popen(
                [
                    "mplayer",
                    "-slave",
                    "-nolirc",
                    "-loop", str(model.repeat_count),
                    "-volume", str(model.volume),
                    model.file_path
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )

            # 5) 응답 반환
            return Response_Sound_PlayT(
                fileName=model.file_name,
                filePath=model.file_path,
                volume=model.volume,
                repeatCount=model.repeat_count,
                result="success",
                message="사운드 재생 완료"
            )

    async def stop(self) -> Response_Sound_StopT:
        """
        [사운드 종료]
        - 현재 재생 중이 아니면 예외 발생
        """
        async with self._lock:
            # 1) 현재 재생 중이 아니면 예외 발생 (혹시 모르니 백그라운드에서 실행되는 모든 mplayer 강제 종료)
            if self.proc is None:
                await self.stop_mplayer()
                raise ServiceException("사운드 재생 중이 아닙니다.", 400)

            # 2) 변수 초기화
            p = self.proc
            self.proc = None
            self.current_sound = None
            self._paused = False
            # 3) watch 스레드 중지
            self._watch_stop.set()

            # 4) terminate 시도
            if p.poll() is None:
                try:
                    p.terminate()
                    await asyncio.wait_for(asyncio.to_thread(p.wait), timeout=0.6)
                except Exception:
                    pass

            # 5) 그래도 안 죽으면 kill
            if p.poll() is None:
                try:
                    p.kill()
                    await asyncio.wait_for(asyncio.to_thread(p.wait), timeout=0.6)
                except Exception:
                    pass

            # 6) 혹시 남아있는 mplayer 정리
            await self.stop_mplayer()

            # 7) 응답 반환
            return Response_Sound_StopT(result="success", message="사운드 종료")

    async def stop_mplayer(self):
        """
        [mplayer 강제 종료]
        - 백그라운드에서 돌고있을 mplayer 프로세스 강제 종료
        """
        try:
            result = subprocess.run("pkill -f mplayer", shell=True, check=False)
            rb_log.debug(f"[sound_linux_adapter] stop_mplayer result: {result}")
        except Exception as e:
            rb_log.warning(f"[sound_linux_adapter] stop_mplayer error: {e}")

    async def pause(self) -> Response_Sound_Pause_ToggleT:
        """
        [사운드 일시정지/재생]
        - 토글입니다. (일시정지 <-> 재생)
        """

        # 1) 현재 재생 중이 아니면 예외 발생
        if self.proc is None:
            raise ServiceException("현재 재생 중인 프로세스가 없습니다.", 400)

        # 2) 명령 전송
        self._send_cmd("pause")

        # 3) 일시정지 상태 변경
        self._paused = not self._paused

        # 4) 응답 반환
        return Response_Sound_Pause_ToggleT(
            result="success",
            message="일시정지" if self._paused else "재생"
        )

    def _send_cmd(self, cmd: str) -> None:
        if self.proc is None or self.proc.stdin is None:
            raise ServiceException("mplayer stdin 사용 불가", 500)
        self.proc.stdin.write(cmd + "\n")
        self.proc.stdin.flush()
