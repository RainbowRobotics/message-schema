import asyncio

_running: set[asyncio.Task] = set()


def spawn(coro, *, name=None, log_ex=True):
    t = asyncio.create_task(coro, name=name)
    _running.add(t)

    def _done(task: asyncio.Task):
        _running.discard(task)
        if log_ex:
            try:
                task.result()
            except Exception as e:
                print(f"[task:{task.get_name() or id(task)}] error: {e}")

    t.add_done_callback(_done)
    return t


async def cancel_all_tasks():
    for t in list(_running):
        t.cancel()
    await asyncio.gather(*_running, return_exceptions=True)


def fire_and_log(coro, *, name=None):
    task = asyncio.create_task(coro, name=name)

    def _done(t: asyncio.Task):
        try:
            t.result()  # 결과를 꺼내면 GC가 안전하게 정리
        except Exception as e:
            print(f"[task:{t.get_name() or id(t)}] error: {e}", flush=True)

    task.add_done_callback(_done)
    return task
