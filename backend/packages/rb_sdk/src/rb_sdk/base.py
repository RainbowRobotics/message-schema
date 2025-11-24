import asyncio
import builtins
import contextlib
import functools
import os
import sys
import threading
import time as time_module
from typing import ClassVar, TypeVar

from rb_schemas.sdk import FlowManagerArgs
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohTransportError

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
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                print(f"[{fn.__name__}] invalid param: {e}", flush=True)
                raise RuntimeError(f"invalid param: {e}") from e
            except (ZenohNoReply, ZenohTransportError) as e:
                print(f"[{fn.__name__}] zenoh error: {e}", flush=True)
                raise RuntimeError(f"zenoh error: {e}") from e
            except asyncio.CancelledError as e:
                raise RuntimeError(f"cancelled: {e}") from e
            except Exception as e:  # noqa: BLE001
                print(f"[{fn.__name__}] unexpected: {e}", flush=True)
                raise RuntimeError(f"unexpected: {e}") from e

        def _sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                print(f"[{fn.__name__}] invalid param: {e}", flush=True)
                raise RuntimeError(f"invalid param: {e}") from e
            except (ZenohNoReply, ZenohTransportError) as e:
                print(f"[{fn.__name__}] zenoh error: {e}", flush=True)
                raise RuntimeError(f"zenoh error: {e}") from e
            except asyncio.CancelledError as e:
                raise RuntimeError(f"cancelled: {e}") from e
            except Exception as e:  # noqa: BLE001
                print(f"[{fn.__name__}] unexpected: {e}", flush=True)
                raise RuntimeError(f"unexpected: {e}") from e

        if asyncio.iscoroutinefunction(fn):
            return functools.wraps(fn)(_async_wrapper)
        return functools.wraps(fn)(_sync_wrapper)

    def __new__(cls) -> "RBBaseSDK":
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

    # async def repeat(self, *, count: int, flow_manager_args: FlowManagerArgs | None = None):
    #     """지정한 횟수만큼 반복하는 함수."""
    #     for i in range(count):
    #         if flow_manager_args is not None:
    #             flow_manager_args.done()

    def close(self):
        """SDK 종료 (현재 프로세스 + 현재 클래스용 인스턴스만)"""
        try:
            # zenoh client 닫기
            if hasattr(self, "zenoh_client") and self.zenoh_client is not None:
                with contextlib.suppress(Exception, TimeoutError):
                    self.zenoh_client.close()

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
