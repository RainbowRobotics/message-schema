import asyncio
import functools
import os
import threading
from typing import ClassVar

from rb_sdk.manipulate import RBManipulateSDK
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError

from .log import rb_log


class BaseService:
    """모든 서비스 공통 에러 핸들링 (sync/async 모두 자동 적용)"""
    manipulate_sdk: RBManipulateSDK
    singleton: ClassVar[bool] = False
    _instances: ClassVar[dict[tuple[int, type["BaseService"]], "BaseService"]] = {}
    _instance_lock: ClassVar[threading.Lock] = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if not getattr(cls, "singleton", False):
            return super().__new__(cls)

        key = (os.getpid(), cls)
        with cls._instance_lock:
            instance = cls._instances.get(key)
            if instance is None:
                instance = super().__new__(cls)
                cls._instances[key] = instance
            return instance

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)

        for name, attr in list(cls.__dict__.items()):
            # private, magic 메서드 제외
            if name.startswith("_"):
                continue

            # property / classmethod / staticmethod는 여기서 제외 (필요하면 따로 처리)
            if isinstance(attr, property | classmethod | staticmethod):
                continue

            # 일반 함수만 감싸기
            if callable(attr):
                wrapped = cls._safe_call(attr)
                setattr(cls, name, wrapped)

    def __init__(self):
        self.manipulate_sdk = RBManipulateSDK()

    @staticmethod
    def _safe_call(fn):
        @functools.wraps(fn)
        async def async_wrapper(*args, **kwargs):
            try:
                return await fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise Exception(e) from e
            except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise Exception(e) from e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise Exception(e) from e

        @functools.wraps(fn)
        def sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise Exception(e) from e
            except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise Exception(e) from e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise Exception(e) from e

        return async_wrapper if asyncio.iscoroutinefunction(fn) else sync_wrapper
