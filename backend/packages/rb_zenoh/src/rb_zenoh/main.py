# zenoh_client.py
import asyncio
import contextlib
import inspect
import json
import os
import sys
import threading
import time
import uuid
from collections.abc import Callable

# import flatbuffers
from typing import Any

import flatbuffers
import psutil
from flatbuffers.table import Table
from utils.parser import t_to_dict
from zenoh import Config, Encoding, QueryTarget, ZBytes, ZError
from zenoh import open as zenoh_open

from rb_zenoh.exeption import ZenohNoReply, ZenohTransportError

from .schema import CallbackEntry, OverflowPolicy, SubscribeOptions
from .utils import invoke, recommend_cap, rough_size_of_fields

# 로그/트레이스 세팅(원하면 조정)
os.environ.setdefault("ZENOH_LOG", "info")
os.environ.setdefault("RUST_LOG", "zenoh=info,zenoh_transport=info,zenoh_shm=info")
os.environ.setdefault("PYTHONUNBUFFERED", "1")
sys.stderr.reconfigure(line_buffering=True)


class ZenohClient:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_init_done", False):
            return
        self._init_done = True

        self.sender = "RRS"
        self.sender_id = str(uuid.uuid4())

        self._mtx = threading.RLock()
        self._owner_pid = os.getpid()
        self._closing = False

        self.session = None

        self._queryables: dict[str, Any] = {}

        self._loop: asyncio.AbstractEventLoop | None = None
        self._schema: dict[str, Callable[[str, memoryview], dict | None]] = {}
        self._subs: dict[str, Any] = {}
        self._cb_entries: dict[str, list[CallbackEntry]] = {}
        self._workers: list[asyncio.Task] = []

        self._fb_stats: dict[tuple[str, str], dict[str, float]] = {}

        self.connect()

    def connect(self):
        if self.session is None:
            conf = Config()
            conf.insert_json5("mode", '"peer"')
            conf.insert_json5("scouting/multicast/enabled", "true")
            conf.insert_json5("scouting/multicast/ttl", "1")
            # conf.insert_json5("scouting/multicast/interface", '"rb_internal"')
            conf.insert_json5("transport/shared_memory/enabled", "true")

            # rb_internal에 글로벌 IPv6/IPv4를 안 주면, 보통 IPv6 링크-로컬(fe80::/64) 만 존재한다.
            conf.insert_json5(
                "listen/endpoints",
                '{ "peer": ["tcp/127.0.0.1:7447", "tcp/zenoh-router:7447", "tcp/[::]:7447#iface=rb_internal"] }',
            )
            conf.insert_json5("listen/exit_on_failure", "false")
            conf.insert_json5("connect/timeout_ms", '{ "peer": -1 }')
            conf.insert_json5(
                "connect/endpoints", '["tcp/127.0.0.1:7447", "tcp/zenoh-router:7447"]'
            )

            delay = 0.5
            for _ in range(8):
                try:
                    self.session = zenoh_open(conf)
                    print("✅ zenoh peer session opened")
                    try:
                        self.is_shm_active()
                    except Exception as e:
                        print(f"[shm-check] skipped: {e}")
                    break
                except ZError as e:
                    print(f"[zenoh] open failed: {e}; retry...")
                    time.sleep(delay)
                    delay = min(delay * 2, 3.0)
            if self.session is None:
                raise RuntimeError("zenoh peer open failed")

            # 디폴트 루프 캐싱(있으면)
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                self._loop = None

    # def publish(self, topic: str, data: Union[bytes, bytearray, memoryview, dict]):
    #     if self.session is None:
    #         self.connect()

    #     message = {
    #             "sender": self.sender,
    #             "sender_id": self.sender_id,
    #             "data": data  # 실질적인 데이터를 data 필드에 담음
    #         }

    #     message_bytes = json.dumps(message).encode("utf-8")

    #     try:
    #         self.session.put(topic, message_bytes)
    #     except Exception as e:
    #         print(f"🚫 zenoh publish failed: {e}")

    def _estimate_initial_size(self, topic: str, fb_class_name: str, fields: dict) -> int:
        key = (topic, fb_class_name)
        stat = self._fb_stats.get(key)
        if stat and "ema" in stat:
            return max(128, int(stat["ema"] * 1.3))

        return max(128, rough_size_of_fields(fields))

    def _update_fb_stats(self, topic: str, fb_class_name: str, payload_len: int):
        key = (topic, fb_class_name)
        s = self._fb_stats.setdefault(key, {})
        ema = s.get("ema")
        if ema is None:
            s["ema"] = payload_len
        else:
            alpha = 0.2
            s["ema"] = int((1 - alpha) * ema + alpha * payload_len)

    def publish(self, topic: str, data: bytes | bytearray | memoryview | dict):
        if self.session is None:
            self.connect()
        payload: bytes
        if isinstance(data, bytes | bytearray | memoryview):
            payload = bytes(data)
        else:
            payload = json.dumps(data).encode("utf-8")

        attachment = f"sender={self.sender};sender_id={self.sender_id}"
        self.session.put(topic, payload, attachment=attachment)

    def subscribe(
        self,
        topic: str,
        callback: Callable[..., Any],
        *,
        flatbuffer_obj_t: Table | None = None,
        options: SubscribeOptions | None = None,
    ):
        """
        topic: 정확히 일치하는 토픽
        callback: (topic, mv, obj, attachment)  sync/async 모두 허용
        반환: handle(해지 시 handle.close())
        """

        if self.session is None:
            self.connect()
        # if "/" not in topic:
        #     raise ValueError("토픽이 명확하지 않습니다. ex) 'telemetry/imu'")

        opts = options or SubscribeOptions()

        if topic not in self._subs:
            sub = self.session.declare_subscriber(
                topic, self._make_on_sample(topic, flatbuffer_obj_t)
            )
            self._subs[topic] = sub
            self._cb_entries.setdefault(topic, [])
            print(f"📥 zenoh subscribed: {topic}")

        entry = CallbackEntry(topic=topic, callback=callback, opts=opts)
        self._cb_entries[topic].append(entry)

        if opts.dispatch == "queue":
            loop = self._ensure_loop()
            task = loop.create_task(self._worker(entry))
            task._zz_entry = entry  # 워커 ↔ 엔트리 연결
            self._workers.append(task)

        return SubscriptionHandleImpl(self, topic, entry)

    async def receive_one(
        self,
        topic: str,
        *,
        flatbuffer_obj_t: Table | None = None,
        timeout: float = 3.0,
        allow_self: bool = False,
        parse_obj: bool = True,
    ) -> tuple[str, memoryview, dict | None, dict]:
        if self.session is None:
            self.connect()

        loop = self._ensure_loop()
        fut = loop.create_future()
        done_once = False

        def _complete(value):
            nonlocal done_once
            if done_once or fut.cancelled() or fut.done():
                return
            done_once = True
            fut.set_result(value)

        def _raw_cb(sample):
            mv = (
                memoryview(bytes(sample.payload))
                if hasattr(sample.payload, "__bytes__")
                else memoryview(sample.payload)
            )
            att = sample.attachment
            if hasattr(att, "to_bytes"):
                att = att.to_bytes().decode("utf-8", "ignore")
            elif isinstance(att, (bytes, bytearray)):
                att = att.decode("utf-8", "ignore")
            parts = dict(seg.split("=", 1) for seg in (att or "").split(";") if "=" in seg)
            sender_id = parts.get("sender_id")
            if not allow_self and sender_id == self.sender_id:
                return

            obj = None
            if flatbuffer_obj_t is not None:
                obj = t_to_dict(flatbuffer_obj_t.InitFromPackedBuf(bytes(mv), 0))
            if parse_obj:
                parser = self._schema.get(topic)
                if parser:
                    try:
                        obj = parser(topic, mv)
                    except Exception:
                        obj = None

            loop.call_soon_threadsafe(
                _complete,
                (topic, mv, obj, {"sender": parts.get("sender"), "sender_id": sender_id}),
            )

        sub = self.session.declare_subscriber(topic, _raw_cb)

        try:
            try:
                return await asyncio.wait_for(fut, timeout=timeout)
            except TimeoutError:
                if not fut.done():
                    fut.cancel()
                raise
        finally:
            with contextlib.suppress(Exception):
                await asyncio.to_thread(sub.undeclare)

    def _unsubscribe(self, topic: str, entry: "CallbackEntry"):
        lst = self._cb_entries.get(topic)
        if lst:
            self._cb_entries[topic] = [e for e in lst if e is not entry]

        # queue 모드 워커만 취소
        for t in list(self._workers):
            if getattr(t, "_zz_entry", None) is entry:
                t.cancel()
                self._workers.remove(t)

        if not self._cb_entries.get(topic):
            try:
                sub = self._subs.pop(topic, None)
                if sub:
                    sub.undeclare()
                    print(f"🧹 zenoh unsubscribed: {topic}")
            except Exception:
                pass
            self._cb_entries.pop(topic, None)

    def unsubscribe_topic(self, topic: str):
        lst = self._cb_entries.pop(topic, [])
        for e in lst:
            for t in list(self._workers):
                if getattr(t, "_zz_entry", None) is e:
                    t.cancel()
                    self._workers.remove(t)
        try:
            sub = self._subs.pop(topic, None)
            if sub:
                sub.undeclare()
                print(f"🧹 zenoh unsubscribed: {topic}")
        except Exception:
            pass

    def unsubscribe_all(self):
        for topic in list(self._cb_entries.keys()):
            self.unsubscribe_topic(topic)

    def queryable(self, keyexpr: str, handler):
        if self.session is None:
            self.connect()

        def _to_bytes(x):
            if x is None:
                return None
            if isinstance(x, bytes | bytearray | memoryview):
                return bytes(x)
            if isinstance(x, str):
                return x.encode("utf-8")
            return json.dumps(x, ensure_ascii=False).encode("utf-8")

        def _auto_check_encoding(original):
            if isinstance(original, bytes | bytearray | memoryview):
                return Encoding.APPLICATION_OCTET_STREAM

            return Encoding.APPLICATION_JSON

        def _handler(q):
            try:
                params = q.parameters or {}
                payload = handler(params)
                if payload is None:
                    q.reply_del(q.key_expr)
                    return
                data = _to_bytes(payload)
                att = f"sender={self.sender};sender_id={self.sender_id}"
                q.reply(
                    key_expr=q.key_expr,
                    payload=ZBytes(data),
                    encoding=_auto_check_encoding(payload),
                    attachment=att,
                )
            except Exception as e:
                q.reply_err(key_expr=q.key_expr, payload=ZBytes(str(e).encode("utf-8")))

        qbl = self.session.declare_queryable(keyexpr, _handler)
        self._queryables[keyexpr] = qbl
        print(f"🛠️  queryable declared: {keyexpr}")

        return qbl

    def undeclare_queryable(self, keyexpr: str):
        qbl = self._queryables.pop(keyexpr, None)
        if qbl:
            with contextlib.suppress(Exception):
                qbl.undeclare()
            print(f"🧹 queryable undeclared: {keyexpr}")

    def query(
        self,
        keyexpr: str,
        *,
        timeout_ms: int = 3000,
        flatbuffer_req_obj: Table | None = None,
        flatbuffer_res_T_class: type[Table] | None = None,
        flatbuffer_buf_size: int | None = None,
        target: QueryTarget | str = "BEST_MATCHING",
        payload: bytes | bytearray | memoryview | None = None,
    ):
        if self.session is None:
            self.connect()

        if flatbuffer_req_obj is not None:
            if flatbuffer_buf_size is None:
                raise ValueError(
                    "flatbuffer_buf_size is required when flatbuffer_obj_t is provided"
                )
            elif flatbuffer_res_T_class is None:
                raise ValueError(
                    "flatbuffer_res_T_class is required when flatbuffer_obj_t is provided"
                )

            b = flatbuffers.Builder(flatbuffer_buf_size)
            b.Finish(flatbuffer_req_obj.Pack(b))
            payload = bytes(b.Output())

        if isinstance(target, str):
            target = {
                "BEST_MATCHING": QueryTarget.BEST_MATCHING,
                "ALL": QueryTarget.ALL,
                "ALL_COMPLETE": QueryTarget.ALL_COMPLETE,
            }.get(target.upper(), QueryTarget.BEST_MATCHING)

        # dict_params = dict(params or {})
        # sel = Selector(keyexpr, dict_params)

        get_result = self.session.get(keyexpr, payload=payload, target=target, timeout=timeout_ms)

        seen_any = False

        try:
            for rep in get_result:
                seen_any = True

                if rep.ok is not None:
                    samp = rep.ok
                    res_payload = (
                        bytes(samp.payload)
                        if isinstance(samp.payload, ZBytes)
                        else bytes(samp.payload)
                    )
                    att = samp.attachment
                    # enc = str(samp.encoding or "flatbuffer").lower()

                    if isinstance(att, bytes | bytearray):
                        att = att.decode("utf-8", "ignore")
                    elif hasattr(att, "to_bytes"):
                        att = att.to_bytes().decode("utf-8", "ignore")

                    att_dict = dict(
                        (k.strip(), v.strip())
                        for seg in (att or "").split(";")
                        if "=" in seg
                        for k, v in [seg.split("=", 1)]
                    )

                    ok_result = {
                        "key": samp.key_expr,
                        "payload": res_payload,
                        "attachment": att_dict,
                        "dict_payload": None,
                        "err": None,
                    }

                    if flatbuffer_req_obj is not None:
                        ok_result["dict_payload"] = t_to_dict(
                            flatbuffer_res_T_class.InitFromPackedBuf(res_payload, 0)
                        )

                        if ok_result["dict_payload"] is None:
                            raise ValueError("dict_payload is not a dict")

                    yield ok_result

                elif rep.err is not None:
                    perr = rep.err
                    res_payload = (
                        bytes(perr.payload)
                        if isinstance(perr.payload, ZBytes)
                        else bytes(perr.payload)
                    )

                    try:
                        msg = res_payload.decode("utf-8")
                    except Exception:
                        msg = res_payload

                    err_result = {
                        "key": samp.key_expr,
                        "payload": res_payload,
                        "attachment": att_dict,
                        "dict_payload": None,
                        "err": msg,
                    }

                    yield err_result

                if not seen_any:
                    raise ZenohNoReply(timeout_ms)

        except ZError as ze:
            raise ZenohTransportError(str(ze)) from ze

    def query_one(
        self,
        keyexpr: str,
        *,
        timeout_ms: int = 3000,
        flatbuffer_req_obj: Table | None = None,
        flatbuffer_res_T_class: type[Table] | None = None,
        flatbuffer_buf_size: int | None = None,
        target: QueryTarget | str = "BEST_MATCHING",
        payload: bytes | bytearray | memoryview | None = None,
    ):
        try:
            return next(
                self.query(
                    keyexpr,
                    flatbuffer_req_obj=flatbuffer_req_obj,
                    flatbuffer_res_T_class=flatbuffer_res_T_class,
                    flatbuffer_buf_size=flatbuffer_buf_size,
                    timeout_ms=timeout_ms,
                    target=target,
                    payload=payload,
                )
            )
        except StopIteration:
            raise ZenohNoReply(timeout_ms) from None

    def query_all(
        self,
        keyexpr: str,
        *,
        timeout_ms: int = 3000,
        flatbuffer_req_obj: Table | None = None,
        flatbuffer_res_T_class: type[Table] | None = None,
        flatbuffer_buf_size: int | None = None,
        target: QueryTarget | str = "BEST_MATCHING",
        payload: bytes | bytearray | memoryview | None = None,
    ):
        res = list(
            self.query(
                keyexpr,
                flatbuffer_req_obj=flatbuffer_req_obj,
                flatbuffer_res_T_class=flatbuffer_res_T_class,
                flatbuffer_buf_size=flatbuffer_buf_size,
                timeout_ms=timeout_ms,
                target=target,
                payload=payload,
            )
        )

        for r in res:
            if flatbuffer_req_obj is not None:
                del r["payload"]
            else:
                del r["dict_payload"]

        return t_to_dict(res)

    def close(self):
        if os.getpid() != getattr(self, "_owner_pid", os.getpid()):
            return

        with self._mtx:
            if getattr(self, "_closing", False):
                return

            self._closing = True

            try:
                for t in list(self._workers):
                    if not t.done():
                        t.cancel()

                self._workers.clear()

                for _topic, sub in list(self._subs.items()):
                    with contextlib.suppress(Exception):
                        sub.undeclare()
                self._subs.clear()
                self._cb_entries.clear()
                self._schema.clear()

                for k in list(self._queryables.keys()):
                    self.undeclare_queryable(k)

                self._queryables.clear()

                sess = getattr(self, "session", None)
                self.session = None

                if sess is not None:
                    with contextlib.suppress(BaseException):
                        sess.close()
                    print("🔌 Zenoh session closed.")
            finally:
                self._closing = False

    def _make_on_sample(self, topic: str, flatbuffer_obj_t: Table | None = None):
        def _on_sample(sample):
            att = sample.attachment
            origin_topic = str(sample.key_expr)

            mv = (
                memoryview(bytes(sample.payload))
                if isinstance(sample.payload, ZBytes)
                else memoryview(sample.payload)
            )

            if hasattr(att, "to_bytes"):
                att = att.to_bytes().decode("utf-8", "ignore")
            elif isinstance(att, bytes | bytearray):
                att = att.decode("utf-8", "ignore")

            parts = dict(
                (k.strip(), v.strip())
                for seg in (att or "").split(";")
                if "=" in seg
                for k, v in [seg.split("=", 1)]
            )

            sender = parts.get("sender")
            sender_id = parts.get("sender_id")

            if sender_id == self.sender_id:
                return

            attachment = {
                "sender": sender,
                "sender_id": sender_id,
            }

            parser = self._schema.get(topic)

            if flatbuffer_obj_t is not None:
                obj = t_to_dict(flatbuffer_obj_t.InitFromPackedBuf(bytes(mv), 0))
            else:
                obj = None

            if parser:
                try:
                    obj = parser(topic, mv)
                except Exception as e:
                    print(f"[parser error:{topic}] {e}", flush=True)

            entries = list(self._cb_entries.get(topic) or [])
            if not entries:
                return

            imms = [e for e in entries if e.opts.dispatch == "immediate"]
            queued = [e for e in entries if e.opts.dispatch == "queue"]

            if imms:
                self._fanout_immediate(
                    entries=imms, topic=origin_topic, mv=mv, obj=obj, attachment=attachment
                )

            for e in queued:
                self._push_to_entry(e=e, topic=origin_topic, mv=mv, obj=obj, attachment=attachment)

        return _on_sample

    def _fanout_immediate(
        self,
        *,
        entries: list["CallbackEntry"],
        topic: str,
        mv: memoryview,
        obj: dict | None,
        attachment: dict,
    ):
        loop = self._ensure_loop()

        async def _spawn_all():
            now = time.time()
            tasks = []

            for e in entries:
                opts = e.opts
                m = e.metrics

                # 샘플링
                if opts.sample_every and opts.sample_every > 1:
                    m["_sample_i"] += 1
                    if (m["_sample_i"] % opts.sample_every) != 0:
                        continue

                # 레이트리밋
                rl_ok = True
                if opts.rate_limit_per_sec:
                    rs = m.setdefault("_rl_start", now)
                    rc = m.setdefault("_rl_count", 0)
                    if now - rs >= 1.0:
                        m["_rl_start"] = now
                        m["_rl_count"] = 0
                        rc = 0
                    if rc >= opts.rate_limit_per_sec:
                        rl_ok = False
                    else:
                        m["_rl_count"] = rc + 1
                if not rl_ok:
                    continue

                if inspect.iscoroutinefunction(e.callback):
                    tasks.append(
                        asyncio.create_task(invoke(e.callback, (topic, mv, obj, attachment)))
                    )
                else:
                    # run_in_executor은 인자 전개가 어려워 래퍼 사용
                    def _sync_call(cb=e.callback, t=topic, r=mv, o=obj, a=attachment):
                        sig = inspect.signature(cb)
                        kwargs = {}

                        if "topic" in sig.parameters:
                            kwargs["topic"] = t
                        if "mv" in sig.parameters:
                            kwargs["mv"] = r
                        if "obj" in sig.parameters:
                            kwargs["obj"] = o
                        if "attachment" in sig.parameters:
                            kwargs["attachment"] = a

                        cb(**kwargs)

                    tasks.append(loop.run_in_executor(None, _sync_call))

                e.metrics["delivered"] += 1
                e.metrics["last_ts"] = now

            if tasks:

                def _done_cb(t: asyncio.Future):
                    exc = t.exception()
                    if exc:
                        print(f"[callback error:{topic}] {exc}", flush=True)

                for t in tasks:
                    if isinstance(t, asyncio.Task):
                        t.add_done_callback(_done_cb)

        if self._closing or loop is None or loop.is_closed():
            return

        loop.call_soon_threadsafe(lambda: asyncio.create_task(_spawn_all()))

    # queue 모드: 엔트리 보호 큐에 푸시
    def _push_to_entry(
        self, *, e: "CallbackEntry", topic: str, mv: memoryview, obj: dict | None, attachment: dict
    ):
        m = e.metrics
        raw_len = len(mv)

        # EMA
        if m.get("ema_bytes") is None:
            m["ema_bytes"] = float(e.opts.expected_avg_bytes or raw_len)
        else:
            alpha = e.opts.ema_alpha
            m["ema_bytes"] = (1.0 - alpha) * float(m["ema_bytes"]) + alpha * float(raw_len)

        # capacity 결정
        if e.opts.maxsize and e.opts.maxsize > 0:
            cap = int(e.opts.maxsize)
        else:
            cap = recommend_cap(m["ema_bytes"], e.opts.mem_budget_mb, e.opts.safety)
        m["capacity"] = cap

        q: asyncio.Queue = e.q

        def _push():
            if q.qsize() < cap:
                try:
                    q.put_nowait((topic, mv, obj, attachment))
                    m["last_qsize"] = q.qsize()
                    m["last_ts"] = time.time()
                    return
                except asyncio.QueueFull:
                    pass

            if e.opts.overflow == OverflowPolicy.DROP_NEW:
                m["dropped_new"] = m.get("dropped_new", 0) + 1
                return

            if e.opts.overflow == OverflowPolicy.DROP_OLDEST:
                try:
                    q.get_nowait()
                    m["dropped_oldest"] = m.get("dropped_oldest", 0) + 1
                    q.put_nowait((topic, mv, obj, attachment))
                    m["last_qsize"] = q.qsize()
                except Exception:
                    m["dropped_new"] = m.get("dropped_new", 0) + 1
                return

            if e.opts.overflow == OverflowPolicy.LATEST_ONLY:
                try:
                    while True:
                        q.get_nowait()
                        m["dropped_latest"] = m.get("dropped_latest", 0) + 1
                except Exception:
                    pass
                try:
                    q.put_nowait((topic, mv, obj, attachment))
                    m["last_qsize"] = q.qsize()
                except Exception:
                    m["dropped_new"] = m.get("dropped_new", 0) + 1
                return

        try:
            self._ensure_loop().call_soon_threadsafe(_push)
        except RuntimeError:
            _push()

    async def _worker(self, e: CallbackEntry):
        q = e.q
        opts = e.opts
        rate_window = 1.0
        rate_count = 0
        rate_start = time.time()

        batch_buf: list[tuple[str, memoryview, dict | None, dict]] = []

        try:
            while True:
                item = await q.get()

                # 배치 X
                if not opts.batch_opts.ms:
                    # 샘플링
                    if opts.sample_every > 1:
                        m = e.metrics
                        m["_sample_i"] = m.get("_sample_i", 0) + 1
                        if (m["_sample_i"] % opts.sample_every) != 0:
                            continue
                    # 레이트리밋
                    if opts.rate_limit_per_sec:
                        now = time.time()
                        if now - rate_start >= rate_window:
                            rate_start = now
                            rate_count = 0
                        if rate_count >= opts.rate_limit_per_sec:
                            continue
                        rate_count += 1
                    await invoke(e.callback, item)
                    e.metrics["delivered"] = e.metrics.get("delivered", 0) + 1
                    continue

                # 배치 O
                batch_buf.append(item)
                started = time.time()
                while len(batch_buf) < opts.batch_opts.max:
                    try:
                        left = (opts.batch_opts.ms / 1000.0) - (time.time() - started)
                        if left <= 0:
                            break
                        nxt = await asyncio.wait_for(q.get(), timeout=left)
                        batch_buf.append(nxt)
                    except TimeoutError:
                        break

                # 샘플링/레이트리밋(배치 수준)
                if opts.sample_every > 1:
                    batch_buf = batch_buf[:: opts.sample_every]
                if opts.rate_limit_per_sec:
                    now = time.time()
                    if now - rate_start >= rate_window:
                        rate_start = now
                        rate_count = 0
                    if rate_count >= opts.rate_limit_per_sec:
                        batch_buf.clear()
                        continue
                    rate_count += 1

                for it in batch_buf:
                    await invoke(e.callback, it)
                    e.metrics["delivered"] = e.metrics.get("delivered", 0) + 1
                batch_buf.clear()
        except asyncio.CancelledError:
            pass

    def _ensure_loop(self) -> asyncio.AbstractEventLoop:
        if self._loop is None:
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                raise RuntimeError(  # noqa: B904
                    "No running asyncio loop; call from within an event loop"
                )  # noqa: B904
        return self._loop

    def is_shm_active(self, topic="diag/shmcheck", seconds=1.0, mb=8):
        if self.session is None:
            return False
        pernic = psutil.net_io_counters(pernic=True)
        lo = next((n for n in ("lo0", "lo", "Loopback Pseudo-Interface 1") if n in pernic), None)

        def nic_bytes():
            if lo:
                c = psutil.net_io_counters(pernic=True)[lo]
                return c.bytes_sent + c.bytes_recv
            return psutil.net_io_counters().bytes_sent + psutil.net_io_counters().bytes_recv

        pub = self.session.declare_publisher(topic)
        sub = self.session.declare_subscriber(topic, lambda _: None)

        before = nic_bytes()
        payload = b"x" * (mb * 1024 * 1024)
        t0 = time.time()
        sent = 0
        while time.time() - t0 < seconds:
            pub.put(payload)
            sent += len(payload)
        time.sleep(0.2)
        after = nic_bytes()

        with contextlib.suppress(Exception):
            sub.undeclare()
        with contextlib.suppress(Exception):
            pub.undeclare()

        net_delta = after - before
        ratio = net_delta / max(1, sent)
        shm_yes = ratio < 0.2
        print(
            f"[SHM-CHECK] sent={sent/1024/1024:.1f}MiB, netΔ={net_delta/1024/1024:.2f}MiB, ratio={ratio:.2f} → SHM={'YES' if shm_yes else 'NO'}"
        )
        return shm_yes


class SubscriptionHandleImpl:
    def __init__(self, client: ZenohClient, topic: str, entry: CallbackEntry):
        self._client = client
        self._topic = topic
        self._entry = entry

    def close(self):
        self._client._unsubscribe(self._topic, self._entry)
