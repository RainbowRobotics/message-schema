import asyncio
import builtins
import contextlib
import functools
import os
import sys
import threading
import time as time_module
from abc import abstractmethod
from typing import Any, ClassVar, Literal, TypeVar

from rb_flat_buffers.program.RB_Program_Dialog import RB_Program_DialogT
from rb_flat_buffers.program.RB_Program_Log import RB_Program_LogT
from rb_flat_buffers.program.RB_Program_Log_Type import RB_Program_Log_Type
from rb_flow_manager.exception import FlowControlException
from rb_modules.log import rb_log
from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import make_builtins_allow_most
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohTransportError

from .schema.base_schema import SetVariableDTO

T = TypeVar("T", bound="RBBaseSDK")


class RBBaseSDK:
    """프로세스별 싱글톤 + 공통 Zenoh/루프 관리"""

    # (pid, cls) 기준으로 인스턴스 관리
    _instances: ClassVar[dict[tuple[int, type["RBBaseSDK"]], "RBBaseSDK"]] = {}
    _lock: ClassVar[threading.Lock] = threading.Lock()

    # 공통 예외 핸들링: 서브클래스 메서드 자동 래핑
    def __init_subclass__(cls, **kwargs):
        """RBBaseSDK를 상속한 모든 클래스의 public instance method에
        공통 try/except를 자동으로 씌움."""
        super().__init_subclass__(**kwargs)

        for name, attr in list(cls.__dict__.items()):
            # _로 시작하는 메서드는 내부용이니까 건드리지 X
            if name.startswith("_"):
                continue

            # property / classmethod / staticmethod는 따로 처리 X
            if isinstance(attr, property | classmethod | staticmethod):
                continue

            # 일반 함수만 래핑
            if callable(attr):
                wrapped = cls._wrap_with_safe_handler(attr)
                setattr(cls, name, wrapped)

    @staticmethod
    def _wrap_with_safe_handler(fn):
        """sync/async 둘 다 지원하는 공통 에러 핸들러"""

        async def _async_wrapper(*args, **kwargs):
            try:
                return await fn(*args, **kwargs)
            except FlowControlException as e:
                raise e
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                print(f"[{fn.__name__}] invalid param: {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e
            except (ZenohNoReply, ZenohTransportError) as e:
                print(f"[{fn.__name__}] zenoh error: {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e
            except asyncio.CancelledError as e:
                raise RuntimeError(f"cancelled: {e}") from e
            except Exception as e:  # noqa: BLE001
                print(f"[{fn.__name__}] {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e

        def _sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except FlowControlException as e:
                raise e
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                print(f"[{fn.__name__}] invalid param: {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e
            except (ZenohNoReply, ZenohTransportError) as e:
                print(f"[{fn.__name__}] zenoh error: {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e
            except asyncio.CancelledError as e:
                raise RuntimeError(f"cancelled: {e}") from e
            except Exception as e:  # noqa: BLE001
                print(f"[{fn.__name__}] {e}", flush=True)
                raise RuntimeError(f"[{fn.__name__}] {e}") from e

        if asyncio.iscoroutinefunction(fn):
            return functools.wraps(fn)(_async_wrapper)
        return functools.wraps(fn)(_sync_wrapper)

    def __new__(cls: type[T]) -> T:
        pid = os.getpid()
        key = (pid, cls)

        with cls._lock:
            if key not in cls._instances:
                instance = super().__new__(cls)
                cls._instances[key] = instance
                instance._initialized = False
            return cls._instances[key]

    def __init__(self):
        # 이미 초기화 됐으면 스킵
        if getattr(self, "_initialized", False):
            return

        self._is_alive = True
        self._initialized = True
        self._pid = os.getpid()

        # 이벤트 루프 & task set
        self.loop = asyncio.new_event_loop()
        self._tasks: set[asyncio.Task] = set()

        # 루프를 돌릴 백그라운드 스레드
        self._loop_thread = threading.Thread(
            target=self._run_loop, name=f"rb-sdk-loop-{self._pid}", daemon=True
        )
        self._loop_thread.start()

        # zenoh client 생성 + 루프 설정
        # self.zenoh_client = ZenohManager.get_client(self.loop)
        self.zenoh_client = ZenohClient()
        self.zenoh_client.set_loop(self.loop)

        print(f"[SDK Base] Initialized for PID {self._pid} ({self.__class__.__name__})")

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        """이벤트 루프에 코루틴 제출하고 future 반환"""

        async def _wrap():
            task = asyncio.current_task()
            if task is not None:
                self._tasks.add(task)
                try:
                    return await coro
                finally:
                    self._tasks.discard(task)

        return asyncio.run_coroutine_threadsafe(_wrap(), self.loop)

    async def wait(self, *, second: float, flow_manager_args: FlowManagerArgs | None = None):
        """지정한 시간만큼 기다리는 함수."""

        ctx = flow_manager_args.ctx if flow_manager_args is not None else None

        # 0 이하 들어오면 그냥 바로 done 처리
        if second <= 0:
            if flow_manager_args is not None:
                flow_manager_args.done()
            else:
                raise ValueError("time must be greater than 0")

        interval = 0.05

        end_at = time_module.monotonic() + second

        while True:
            # 정지 요청(PAUSE/STOP 등) 들어왔는지 확인
            if ctx is not None:
                ctx.check_stop()

            now = time_module.monotonic()
            remaining = end_at - now

            if remaining <= 0:
                break

            sleep_for = interval if remaining > interval else remaining
            await asyncio.sleep(sleep_for)

        if flow_manager_args is not None:
            flow_manager_args.done()

    def set_variables(self, *, variables: list[SetVariableDTO], flow_manager_args: FlowManagerArgs | None = None):
        """변수 설정"""
        if flow_manager_args is not None:
            for variable in variables:
                flow_manager_args.ctx.update_local_variables({
                    variable["name"]: variable["init_value"]
                })
            flow_manager_args.done()

    def make_script(self, *, contents: str, flow_manager_args: FlowManagerArgs | None = None):
        """스크립트 생성"""
        code = compile(contents, "<custom_script>", "exec")


        merged_variables = {
            **(flow_manager_args.ctx.variables.get("global") or {}),
            **(flow_manager_args.ctx.variables.get("local") or {}),
        }

        env = {
            "variables": merged_variables,
            "var": merged_variables,
            "update_variable": flow_manager_args.ctx.update_local_variables,
            "done": flow_manager_args.done,
            "pause": flow_manager_args.ctx.pause,
            "stop": flow_manager_args.ctx.stop,
            "resume": flow_manager_args.ctx.resume,
            "check_stop": flow_manager_args.ctx.check_stop,
            "rb_log": rb_log,
        }

        try:
            exec(  # pylint: disable=exec-used
                code,
                {"__builtins__": make_builtins_allow_most()},
                env,
            )
        finally:
            flow_manager_args.done()


    def all_pause(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """모든 프로세스 일시정지"""
        self.zenoh_client.publish("rrs/pause", payload={})

        time_module.sleep(0.1)

        if flow_manager_args is not None:
            flow_manager_args.ctx.pause()

    def all_stop(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """모든 프로세스 정지"""
        self.zenoh_client.publish("rrs/stop", payload={})
        if flow_manager_args is not None:
            flow_manager_args.done()

    def alarm(self, *, title: str, content: str, robot_model: str, flow_manager_args: FlowManagerArgs | None = None):
        """프로그램 알림 발생"""
        req = RB_Program_DialogT()
        req.robotModel = robot_model
        req.title = title or "No Title"
        req.content = content

        self.zenoh_client.publish(
            "rrs/program/dialog",
            flatbuffer_req_obj=req,
            flatbuffer_buf_size=256,
        )


        if flow_manager_args is not None:
            flow_manager_args.done()

    def log(
        self,
        *,
        content: str,
        robot_model: str,
        level: Literal["INFO", "WARNING", "ERROR", "USER", "DEBUG", "GENERAL"],
        flow_manager_args: FlowManagerArgs | None = None):
        """프로그램 로그 발생"""
        req = RB_Program_LogT()
        req.content = content
        req.robotModel = robot_model

        if level == "INFO":
            req.type = RB_Program_Log_Type.INFO
        elif level == "WARNING":
            req.type = RB_Program_Log_Type.WARNING
        elif level == "ERROR":
            req.type = RB_Program_Log_Type.ERROR
        elif level == "USER":
            req.type = RB_Program_Log_Type.USER
        elif level == "DEBUG":
            req.type = RB_Program_Log_Type.DEBUG
        elif level == "GENERAL":
            req.type = RB_Program_Log_Type.GENERAL

        self.zenoh_client.publish(
            "rrs/program/log",
            flatbuffer_req_obj=req,
            flatbuffer_buf_size=256,
        )

        if flow_manager_args is not None:
            flow_manager_args.done()


    # async def repeat(self, *, count: int, flow_manager_args: FlowManagerArgs | None = None):
    #     """지정한 횟수만큼 반복하는 함수."""
    #     for i in range(count):
    #         if flow_manager_args is not None:
    #             flow_manager_args.done()

    @abstractmethod
    async def set_begin(
        self,
        *,
        robot_model: str,
        position: Any,
        is_enable: bool = True,
        speed_ratio: float | None = None,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """메인 태스크 시작 위치 설정"""


    def close(self):
        """SDK 종료 (현재 프로세스 + 현재 클래스용 인스턴스만)"""
        try:
            # zenoh client 닫기
            if hasattr(self, "zenoh_client") and self.zenoh_client is not None:
                with contextlib.suppress(Exception, TimeoutError):
                    self.zenoh_client.close()
            # ZenohManager.close_local()

            # 루프 위에서 돌고 있는 task 취소
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

                # 루프 정지
                self.loop.call_soon_threadsafe(self.loop.stop)

            # 스레드 join
            if getattr(self, "_loop_thread", None) and self._loop_thread.is_alive():
                self._loop_thread.join(timeout=2)

            # 루프 close
            if hasattr(self, "loop") and self.loop is not None and not self.loop.is_closed():
                self.loop.close()

            self._is_alive = False

            print(f"[SDK Base] Closed for PID {self._pid} ({self.__class__.__name__})")

        except Exception as e:
            print(f"[SDK Base] Close error: {e}", flush=True)
        finally:
            cls = self.__class__
            key = (self._pid, cls)
            with cls._lock:
                if cls._instances.get(key) is self:
                    cls._instances.pop(key, None)

    def __del__(self):
        """GC 시 안전 종료 (파이썬 종료 중이면 무시)"""
        if getattr(sys, "is_finalizing", lambda: False)():
            return

        if os.getpid() == getattr(self, "_pid", None):
            with contextlib.suppress(builtins.BaseException):
                self.close()
