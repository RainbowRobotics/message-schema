import asyncio
from collections import defaultdict
from typing import Any

from .framing import pack


class Registry:
    """ registry for tcp connections """

    def __init__(self, *, max_sub_per_conn: int = 100, drain_timeout: float = 0.5):
        self._topic_map: dict[str, set[asyncio.StreamWriter]] = defaultdict(set)
        self._conn_topics: dict[asyncio.StreamWriter, set[str]] = defaultdict(set)
        self._lock = asyncio.Lock()
        self._max_sub = max_sub_per_conn
        self._drain_timeout = drain_timeout

    async def sub(self, topic: str, w: asyncio.StreamWriter):
        """ subscribe to a topic """
        async with self._lock:
            topics = self._conn_topics[w]
            if len(topics) >= self._max_sub:
                raise ValueError("subscription_limit_exceeded")
            topics.add(topic)
            self._topic_map[topic].add(w)

    async def unsub(self, topic: str, w: asyncio.StreamWriter):
        """ unsubscribe from a topic """
        async with self._lock:
            self._topic_map.get(topic, set()).discard(w)
            self._conn_topics.get(w, set()).discard(topic)

    async def unsub_all(self, w: asyncio.StreamWriter):
        """ unsubscribe from all topics """
        async with self._lock:
            topics = self._conn_topics.pop(w, set())
            for t in topics:
                self._topic_map.get(t, set()).discard(w)

    async def push(self, topic: str, data: dict[str, Any]):
        """ push data to a topic """
        payload = pack({"type": "evt", "topic": topic, "data": data})

        async with self._lock:
            writers = list(self._topic_map.get(topic, set()))

        dead: list[asyncio.StreamWriter] = []

        for w in writers:
            try:
                w.write(payload)
                await asyncio.wait_for(w.drain(), timeout=self._drain_timeout)
            except Exception:
                dead.append(w)

        if dead:
            async with self._lock:
                for w in dead:
                    await self._remove_writer(w)

    async def _remove_writer(self, w: asyncio.StreamWriter):
        topics = self._conn_topics.pop(w, set())
        for t in topics:
            self._topic_map.get(t, set()).discard(w)
