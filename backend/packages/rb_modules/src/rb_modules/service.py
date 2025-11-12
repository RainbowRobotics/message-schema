import asyncio
import functools

from rb_zenoh.exeption import (
    ZenohNoReply,
    ZenohTransportError,
)

from .log import rb_log


class BaseService:
    """모든 서비스 공통 에러 핸들링 (sync/async 둘 다 자동 적용)"""

    def __init__(self):
        self._wrap_all_methods_with_safe_handler()

    def _safe_call(self, fn):
        @functools.wraps(fn)
        async def async_wrapper(*args, **kwargs):
            try:
                return await fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise e
            except (ZenohNoReply, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise e

        @functools.wraps(fn)
        def sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise e
            except (ZenohNoReply, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise e

        if asyncio.iscoroutinefunction(fn):
            return async_wrapper
        return sync_wrapper

    def _wrap_all_methods_with_safe_handler(self):
        """비공개 메서드 제외, 모든 callable을 자동 예외 핸들링"""
        for name in dir(self):
            if name.startswith("_"):
                continue
            fn = getattr(self, name)
            if callable(fn):
                setattr(self, name, self._safe_call(fn))
