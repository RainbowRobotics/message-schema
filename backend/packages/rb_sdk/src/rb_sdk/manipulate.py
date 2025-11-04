import asyncio
import builtins
import contextlib
import os
import sys
import threading
from typing import ClassVar

from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_Move_J import Request_Move_JT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_schemas.sdk import FlowManagerArgs
from rb_zenoh.client import ZenohClient


class RBManipulateSDK:
    """멀티프로세스 안전한 RB Manipulate SDK

    각 프로세스마다 독립적인 인스턴스를 생성하여
    멀티프로세스 환경에서 안전하게 동작합니다.
    """

    # 프로세스별 인스턴스 저장
    _instances: ClassVar[dict[int, "RBManipulateSDK"]] = {}
    _lock: ClassVar[threading.Lock] = threading.Lock()

    def __new__(cls):
        """프로세스별로 싱글톤 인스턴스 생성"""
        pid = os.getpid()

        with cls._lock:
            if pid not in cls._instances:
                instance = super().__new__(cls)
                cls._instances[pid] = instance
                instance._initialized = False

            return cls._instances[pid]

    def __init__(self):
        """프로세스별 초기화 (한 번만 실행)"""
        # 이미 초기화되었으면 스킵
        if getattr(self, "_initialized", False):
            return

        self._initialized = True
        self._pid = os.getpid()

        # 이벤트 루프 생성
        self.loop = asyncio.new_event_loop()

        self._tasks: set[asyncio.Task] = set()

        # 별도 스레드에서 이벤트 루프 실행
        self._loop_thread = threading.Thread(
            target=self._run_loop, name=f"rb-sdk-loop-{self._pid}", daemon=True
        )
        self._loop_thread.start()

        # zenoh client 생성 및 루프 설정
        self.zenoh_client = ZenohClient()
        self.zenoh_client.set_loop(self.loop)

        print(f"[SDK] Initialized for PID {self._pid}")

    def _run_loop(self):
        """백그라운드 스레드에서 이벤트 루프 실행"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        """이벤트 루프에 코루틴 제출"""

        async def _wrap():
            task = asyncio.current_task()
            if task is not None:
                self._tasks.add(task)
                try:
                    return await coro
                finally:
                    self._tasks.discard(task)

        return asyncio.run_coroutine_threadsafe(_wrap(), self.loop)

    async def move_j(
        self,
        robot_model: str,
        joints: list[float],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """관절 공간 이동 명령

        Args:
            robot_model: 로봇 모델명
            joints: 관절 각도 리스트
            flow_manager_args: Flow Manager 인자 (done 콜백 등)
        """
        try:
            # flatbuffer 요청 구성
            req = Request_Move_JT()
            move_input_target = MoveInput_TargetT()
            move_input_speed = MoveInput_SpeedT()

            ni = N_INPUT_fT()
            ni.f = joints

            move_input_target.tarValues = ni
            move_input_target.tarFrame = -1
            move_input_target.tarUnit = 0

            move_input_speed.spdMode = 1
            move_input_speed.spdVelPara = 60
            move_input_speed.spdAccPara = 120

            req.target = move_input_target
            req.speed = move_input_speed

            print("req >>", req, flush=True)

            # 명령 전송
            res = self.zenoh_client.query_one(
                f"{robot_model}/call_move_j",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=256,
            )

            # 상태 구독 (flow_manager_args가 있는 경우)
            if flow_manager_args is not None and res.get("dict_payload"):
                while True:
                    try:
                        topic, mv, obj, attachment = await self.zenoh_client.receive_one(
                            f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                        )

                        if obj is not None and obj.get("motionMode") == 0:
                            flow_manager_args.done()
                            break
                    except asyncio.CancelledError:
                        print("CancelledError", flush=True)
                        break
                    except Exception as e:
                        print("Exception >>", e, flush=True)
                        raise RuntimeError(e) from e

        except Exception as e:
            if flow_manager_args is not None:
                raise RuntimeError(f"Move failed: {e}") from e

    def _on_state_core(self, obj: dict, flow_manager_args: FlowManagerArgs):
        """State Core 콜백 처리"""
        if obj and obj.get("motionMode") == 0:
            # 모션 완료
            flow_manager_args.done()

    def move_l(
        self,
        robot_model: str,
        position: list[float],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """직선 공간 이동 명령 (예시)"""
        # 구현 필요
        pass

    def close(self):
        """SDK 종료 (현재 프로세스만)"""
        try:
            if hasattr(self, "zenoh_client") and self.zenoh_client is not None:
                with contextlib.suppress(Exception, TimeoutError):
                    self.zenoh_client.close()

            if hasattr(self, "loop") and self.loop is not None and not self.loop.is_closed():

                async def _cancel_all():
                    tasks = list(self._tasks)
                    for t in tasks:
                        t.cancel()
                    if tasks:
                        await asyncio.gather(*tasks, return_exceptions=True)

                fut = asyncio.run_coroutine_threadsafe(_cancel_all(), self.loop)
                with contextlib.suppress(Exception):
                    fut.result(timeout=2)

                self.loop.call_soon_threadsafe(self.loop.stop)

            if getattr(self, "_loop_thread", None) and self._loop_thread.is_alive():
                self._loop_thread.join(timeout=2)

            if hasattr(self, "loop") and self.loop is not None and not self.loop.is_closed():
                self.loop.close()

            print(f"[SDK] Closed for PID {self._pid}")
        except Exception as e:
            print(f"[SDK] Close error: {e}")
        finally:
            cls = self.__class__
            with cls._lock:  # pylint: disable=protected-access
                if cls._instances.get(self._pid) is self:  # pylint: disable=protected-access
                    cls._instances.pop(self._pid, None)  # pylint: disable=protected-access

    def __del__(self):
        """소멸자"""
        if getattr(sys, "is_finalizing", lambda: False)():
            return

        if os.getpid() == getattr(self, "_pid", None):
            with contextlib.suppress(builtins.BaseException):
                self.close()
