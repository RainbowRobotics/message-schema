import asyncio
from collections.abc import Awaitable
from typing import Any

from rb_modules.log import rb_log

_running: set[asyncio.Task] = set()

_priv_loop = None
_priv_thread = None


def spawn(coro, *, name=None, log_ex=True):
    t = asyncio.create_task(coro, name=name)
    _running.add(t)

    def _done(task: asyncio.Task):
        _running.discard(task)
        if log_ex:
            try:
                task.result()
            except Exception as e:
                rb_log.error(f"[task:{task.get_name() or id(task)}] error: {e}")

    t.add_done_callback(_done)
    return t


async def cancel_all_tasks():
    for t in list(_running):
        t.cancel()
    await asyncio.gather(*_running, return_exceptions=True)


def _ensure_private_loop() -> asyncio.AbstractEventLoop:
    global _priv_loop, _priv_thread
    if _priv_loop and _priv_loop.is_running():
        return _priv_loop
    loop = asyncio.new_event_loop()
    _priv_loop = loop

    import threading

    def _runner():
        asyncio.set_event_loop(loop)
        loop.run_forever()

    _priv_thread = threading.Thread(target=_runner, name="fire-and-log-loop", daemon=True)
    _priv_thread.start()
    return loop


def fire_and_log(
    coro: Awaitable[Any],
    *,
    name: str | None = None,
    loop: asyncio.AbstractEventLoop | None = None,
):
    try:
        running = asyncio.get_running_loop()

        if loop is not None and loop is not running:
            fut = asyncio.run_coroutine_threadsafe(coro, loop)
            _attach_done_logging(fut, name=name)
            return fut

        task: asyncio.Task = running.create_task(coro, name=name)  # type: ignore
        _attach_done_logging(task, name=name)
        return task
    except RuntimeError:
        target_loop = loop
        if target_loop is None:
            target_loop = _ensure_private_loop()

        fut = asyncio.run_coroutine_threadsafe(coro, target_loop)
        _attach_done_logging(fut, name=name)
        return fut


def _attach_done_logging(fut_or_task, *, name: str | None):
    def _done(f):
        try:
            _ = f.result()
        except asyncio.CancelledError:
            rb_log.info(f"[task:{name or getattr(f, 'get_name', lambda: '')() or id(f)}] cancelled")
        except Exception:
            rb_log.error(f"[task:{name or getattr(f, 'get_name', lambda: '')() or id(f)}] crashed")

    fut_or_task.add_done_callback(_done)
