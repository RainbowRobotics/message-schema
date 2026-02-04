import asyncio
import builtins
import contextlib
import functools
import os
import sys
import threading
import time as time_module
from collections.abc import Callable
from typing import Any, ClassVar, Literal, TypeVar

from rb_flat_buffers.program.RB_Program_Dialog import RB_Program_DialogT
from rb_flat_buffers.program.RB_Program_Log import RB_Program_LogT
from rb_flat_buffers.program.RB_Program_Log_Type import RB_Program_Log_Type
from rb_flow_manager.exception import FlowControlException
from rb_modules.log import rb_log
from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import make_builtins_allow_most, safe_eval_expr
from rb_zenoh.client import FBRootReadable, ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohTransportError
from rb_zenoh.schema import SubscribeOptions

from .schema.base_schema import SetVariableDTO


class VariablesProxy:
    """변수 프록시"""

    def __init__(self, ctx: Any):
        self._ctx = ctx

    def __getitem__(self, key: str):
        local = self._ctx.variables.get("local", {})
        if key in local:
            return local[key]

        global_vars = self._ctx.variables.get("global", {})
        if key in global_vars:
            return global_vars[key]

        if key.startswith("RB_"):
            value = self._ctx.get_global_variable(key)
            if value is None:
                raise KeyError(key)
            return value

        raise KeyError(key)

    def get(self, key: str, default: Any | None = None):
        try:
            return self[key]
        except KeyError:
            return default


T = TypeVar("T", bound="RBBaseSDK")


class RBBaseSDK:
    """프로세스별 싱글톤 + 공통 Zenoh/루프 관리 + 참조 카운팅"""

    # (pid, cls) 기준으로 인스턴스 관리
    _instances: ClassVar[dict[tuple[int, type["RBBaseSDK"]], "RBBaseSDK"]] = {}

    # PID별 전체 SDK 참조 카운트 (모든 SDK 클래스 통합)
    _total_ref_counts: ClassVar[dict[int, int]] = {}

    # PID별 ZenohClient 인스턴스
    _zenoh_clients: ClassVar[dict[int, ZenohClient]] = {}

    _lock: ClassVar[threading.Lock] = threading.Lock()

    # 공통 예외 핸들링: 서브클래스 메서드 자동 래핑
    def __init_subclass__(cls, **kwargs):
        """RBBaseSDK를 상속한 모든 클래스의 public instance method에
        공통 try/except를 자동으로 씌움."""
        super().__init_subclass__(**kwargs)

        for name, attr in list(cls.__dict__.items()):
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
                print(f"[{fn.__name__}] cancelled error: {e}", flush=True)
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

        with RBBaseSDK._lock:
            if key not in RBBaseSDK._instances:
                instance = super().__new__(cls)
                RBBaseSDK._instances[key] = instance
                instance._initialized = False

                # ✅ 데코레이터보다 먼저 초기화
                instance._zenoh_subscribers = []
                instance._zenoh_queryables = []
                instance._pid = pid
                instance._closing = False

                # 카운트 증가
                RBBaseSDK._total_ref_counts[pid] = RBBaseSDK._total_ref_counts.get(pid, 0) + 1

                # ZenohClient 생성 및 설정
                if pid not in RBBaseSDK._zenoh_clients:
                    RBBaseSDK._zenoh_clients[pid] = ZenohClient()

                # ✅ 즉시 설정
                instance.zenoh_client = RBBaseSDK._zenoh_clients[pid]

            return RBBaseSDK._instances[key]

    def __init__(self, *, server: Literal["amr", "manipulate"] | None = None):
        # 이미 초기화 됐으면 스킵
        if getattr(self, "_initialized", False):
            return

        self._is_alive = True
        self._pid = os.getpid()
        self._closing = False

        # 리스트 초기화 (데코레이터가 먼저 실행될 수 있음)
        if not hasattr(self, "_zenoh_queryables"):
            self._zenoh_queryables = []
        if not hasattr(self, "_zenoh_subscribers"):
            self._zenoh_subscribers = []

        # 공유 ZenohClient 사용
        with RBBaseSDK._lock:
            self.zenoh_client = RBBaseSDK._zenoh_clients[self._pid]

        self.robot_uids = self._get_robot_uids(server=server) if server is not None else {}

        self._initialized = True

        print(f"[SDK Base] Initialized {self.__class__.__name__} for PID {self._pid} (total refs: {RBBaseSDK._total_ref_counts[self._pid]})", flush=True)

    def _get_robot_uids(self, *, server: Literal["amr", "manipulate"]) -> dict[str, str]:
        """
        server_type 기준으로 robot_uid를 질의해서 가져온다.
        없으면 None.
        """
        try:
            result = self.zenoh_client.query_all(
                f"rrs/{server}/robot_uid",
                payload={},
                timeout=0.1,
            )


            return result.get("dict_payload") or {}
        except Exception as e:
            self.log(content=f"[SDK Base] Error getting robot uid: {e}", robot_model="RRS", level="ERROR")
            return {}

    def register_zenoh_queryable(self, keyexpr: str, queryable):
        """Queryable 등록 (keyexpr와 함께 저장)"""
        self._zenoh_queryables.append((keyexpr, queryable))
        return queryable

    def register_zenoh_subscriber(self, subscriber):
        """Subscriber 등록 (정리를 위해)"""
        self._zenoh_subscribers.append(subscriber)
        return subscriber

    def zenoh_subscribe(
        self,
        topic: str,
        *,
        flatbuffer_obj_t: FBRootReadable | None = None,
        opts: SubscribeOptions | None = None,
    ):
        """데코레이터: Subscribe 자동 등록 및 정리"""
        def decorator(callback: Callable):
            # ✅ _initialized 체크 제거

            handle = self.zenoh_client.subscribe(
                topic=topic,
                callback=callback,
                flatbuffer_obj_t=flatbuffer_obj_t,
                options=opts,
            )

            # 리스트가 없으면 생성
            if not hasattr(self, "_zenoh_subscribers"):
                self._zenoh_subscribers = []

            self._zenoh_subscribers.append(handle)
            return callback
        return decorator

    def zenoh_queryable(
        self,
        keyexpr: str,
        *,
        flatbuffer_req_T_class: FBRootReadable | None = None,
        flatbuffer_res_buf_size: int | None = None,
    ):
        """데코레이터: Queryable 자동 등록 및 정리"""
        def decorator(handler: Callable):
            # ✅ _initialized 체크 제거

            # 중복 체크
            if keyexpr in self.zenoh_client._queryables:
                print(f"⚠️  Queryable already declared: {keyexpr}", flush=True)
                return handler  # 중복이면 그냥 반환

            # Queryable 등록
            qbl = self.zenoh_client.queryable(
                keyexpr=keyexpr,
                handler=handler,
                flatbuffer_req_T_class=flatbuffer_req_T_class,
                flatbuffer_res_buf_size=flatbuffer_res_buf_size,
            )

            # 리스트가 없으면 생성
            if not hasattr(self, "_zenoh_queryables"):
                self._zenoh_queryables = []

            self._zenoh_queryables.append((keyexpr, qbl))

            return handler
        return decorator

    def _cleanup_resources(self):
        """이 SDK 인스턴스의 Zenoh 리소스 정리"""
        print(f"[SDK Base] Cleaning up resources for {self.__class__.__name__}", flush=True)

        # Queryable 정리
        for keyexpr, _ in self._zenoh_queryables:
            try:
                self.zenoh_client.undeclare_queryable(keyexpr)
            except Exception as e:
                print(f"[SDK Base] Error undeclaring queryable {keyexpr}: {e}", flush=True)
        self._zenoh_queryables.clear()

        # Subscriber 정리
        for zenoh_subscriber in self._zenoh_subscribers:
            try:
                zenoh_subscriber.close()
            except Exception as e:
                print(f"[SDK Base] Error closing subscriber: {e}", flush=True)
        self._zenoh_subscribers.clear()

    def _run_coro_blocking(self, coro, *, timeout: float | None = None):
        try:
            loop = asyncio.get_running_loop()
            # running loop가 있으면 blocking하면 안 됨
            loop.create_task(coro)
            return None
        except RuntimeError:
            # 루프가 없으면 새 루프에서 실행
            if timeout is None:
                return asyncio.run(coro)

            async def _with_timeout():
                return await asyncio.wait_for(coro, timeout=timeout)

            return asyncio.run(_with_timeout())

    def set_variables(self, *, variables: list[SetVariableDTO], flow_manager_args: FlowManagerArgs | None = None):
        """변수 설정"""
        if flow_manager_args is not None:
            for variable in variables:
                flow_manager_args.ctx.update_local_variables({
                    variable["name"]: variable["init_value"]
                })
            flow_manager_args.done()

    def make_script(self, *, mode: Literal["GENERAL", "ADVANCED"] = "ADVANCED", contents: str | None = None, flow_manager_args: FlowManagerArgs | None = None):
        """스크립트 생성"""

        general_contents = flow_manager_args.args.get("general_contents", None)

        if mode == "GENERAL":
            if general_contents is None:
                raise RuntimeError("general_contents is required")

            vars_ = flow_manager_args.ctx.variables

            # ctx.variables가 {"c": ...} 같은 평평한 dict면 local로 감싸기
            if vars_ is not None and "local" not in vars_ and "global" not in vars_:
                vars_ = {"local": vars_}

            for content in general_contents:
                safe_eval_expr(
                    content,
                    variables=vars_,
                    get_global_variable=flow_manager_args.ctx.get_global_variable,
                )

            flow_manager_args.done()
        elif mode == "ADVANCED":
            if contents is None:
                raise RuntimeError("contents is required")

            code = compile(contents, "<custom_script>", "exec")

            merged_variables = VariablesProxy(flow_manager_args.ctx)

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

    def all_pause(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """모든 프로세스 일시정지"""
        self.zenoh_client.query_one("rrs/pause", payload={})

        time_module.sleep(0.1)

        if flow_manager_args is not None:
            flow_manager_args.ctx.pause()

    def all_stop(self, *, flow_manager_args: FlowManagerArgs | None = None):
        """모든 프로세스 정지"""
        self.zenoh_client.query_one("rrs/stop", payload={})
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

    def close(self):
        """SDK 종료 (전체 참조 카운트 기반으로 ZenohClient 정리 여부 결정)"""
        if os.getpid() != getattr(self, "_pid", os.getpid()):
            return

        pid = self._pid
        key = (pid, self.__class__)

        should_close_zenoh = False
        should_cleanup_resources = False

        with RBBaseSDK._lock:
            # 중복 close 방지
            if getattr(self, "_closing", False):
                self.log(content=f"[SDK Base] {self.__class__.__name__} (PID {pid}) close already in progress", robot_model="RRS", level="DEBUG")
                return

            # 이미 제거된 인스턴스는 건너뜀 (카운트 감소 X)
            if key not in RBBaseSDK._instances:
                self.log(content=f"[SDK Base] {self.__class__.__name__} (PID {pid}) already removed, skipping", robot_model="RRS", level="DEBUG")
                return

            self._closing = True

            # 전체 참조 카운트 감소
            if pid in RBBaseSDK._total_ref_counts:
                RBBaseSDK._total_ref_counts[pid] -= 1
                remaining = RBBaseSDK._total_ref_counts[pid]

                self.log(content=f"[SDK Base] Closing {self.__class__.__name__} (PID {pid}), remaining SDKs: {remaining}", robot_model="RRS", level="DEBUG")

                # 이 클래스의 인스턴스가 정리될 때 리소스 정리
                if key in RBBaseSDK._instances:
                    should_cleanup_resources = True
                    RBBaseSDK._instances.pop(key, None)

                # 모든 SDK가 정리되었으면 ZenohClient도 닫기
                if remaining <= 0:
                    should_close_zenoh = True
                    RBBaseSDK._total_ref_counts.pop(pid, None)
                    self.log(content=f"[SDK Base] ✅ All SDKs closed for PID {pid}, will close ZenohClient", robot_model="RRS", level="DEBUG")

        # Lock 밖에서 리소스 정리 (deadlock 방지)
        if should_cleanup_resources:
            try:
                self._cleanup_resources()
                self.log(content=f"[SDK Base] ✅ Resources cleaned for {self.__class__.__name__}", robot_model="RRS", level="DEBUG")
            except Exception as e:
                self.log(content=f"[SDK Base] Error cleaning up resources: {e}", robot_model="RRS", level="ERROR")

        # Lock 밖에서 ZenohClient 정리 (deadlock 방지)
        if should_close_zenoh:
            try:
                with RBBaseSDK._lock:
                    zenoh_client = RBBaseSDK._zenoh_clients.pop(pid, None)

                if zenoh_client is not None:
                    self.log(content=f"[SDK Base] ✅ ZenohClient closed for PID {pid}", robot_model="RRS", level="DEBUG")
                    with contextlib.suppress(Exception, TimeoutError):
                        zenoh_client.close()

            except Exception as e:
                print(f"[SDK Base] Error closing ZenohClient: {e}", flush=True)

        self._is_alive = False
        self._closing = False

    def __del__(self):
        """GC 시 안전 종료 (파이썬 종료 중이면 무시)"""
        if getattr(sys, "is_finalizing", lambda: False)():
            return

        if os.getpid() == getattr(self, "_pid", None):
            with contextlib.suppress(builtins.BaseException):
                self.close()
