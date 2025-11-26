import asyncio
import functools

from rb_zenoh.exeption import ZenohNoReply, ZenohTransportError

from .log import rb_log


class BaseService:
    """모든 서비스 공통 에러 핸들링 (sync/async 모두 자동 적용)"""

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

    @staticmethod
    def _safe_call(fn):
        @functools.wraps(fn)
        async def async_wrapper(*args, **kwargs):
            try:
                return await fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise RuntimeError(e) from e
            except (ZenohNoReply, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise RuntimeError(e) from e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise RuntimeError(e) from e

        @functools.wraps(fn)
        def sync_wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except (TypeError, ValueError, AttributeError, KeyError) as e:
                rb_log.error(f"[{fn.__name__}] invalid param: {e}")
                raise RuntimeError(e) from e
            except (ZenohNoReply, ZenohTransportError) as e:
                rb_log.error(f"[{fn.__name__}] zenoh error: {e}")
                raise RuntimeError(e) from e
            except Exception as e:
                rb_log.error(f"[{fn.__name__}] unexpected: {e}")
                raise RuntimeError(e) from e

        return async_wrapper if asyncio.iscoroutinefunction(fn) else sync_wrapper
