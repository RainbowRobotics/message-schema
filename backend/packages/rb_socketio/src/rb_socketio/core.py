import asyncio
import threading
from collections.abc import Awaitable, Callable
from typing import ClassVar, Optional

import socketio
from fastapi import FastAPI
from socketio import AsyncRedisManager

from .config import SocketConfig


class RBSocketIO:
    _instance: ClassVar[Optional["RBSocketIO"]] = None
    _lock: ClassVar[asyncio.Lock] = threading.Lock()

    def __init__(
        self,
        conf: SocketConfig,
        *,
        engineio_path: str,
        fastapi_app: FastAPI,
        on_connect: Callable[[str, dict, dict | None], Awaitable[None]] = None,
        on_disconnect: Callable[[str], Awaitable[None]] = None,
    ):
        self.conf = conf
        # self.jwt = JwtAuth(conf)

        mgr = None
        if conf.redis_url and AsyncRedisManager:
            mgr = AsyncRedisManager(conf.redis_url)

        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            client_manager=mgr,
            logger=conf.logger,
            cors_allowed_origins=conf.cors_origins,
            ping_interval=conf.ping_interval,
            ping_timeout=conf.ping_timeout,
            allow_upgrades=conf.allow_upgrades,
        )
        self.asgi = socketio.ASGIApp(
            self.sio, other_asgi_app=fastapi_app, socketio_path=engineio_path
        )

        @self.sio.event(namespace=self.conf.namespace)
        async def connect(sid, environ, auth):
            # token = self.jwt.pick_token(auth, environ)
            # if not token:
            #     return False
            # try:
            #     claims = self.jwt.verify(token)
            # except Exception:
            #     return False
            # await self.sio.save_session(sid, {"claims": claims})

            if on_connect:
                print("on_connect", flush=True)
                await on_connect(sid, environ, auth)

        @self.sio.event(namespace=self.conf.namespace)
        async def disconnect(sid):
            if on_disconnect:
                await on_disconnect(sid)

    @classmethod
    def init(
        cls,
        conf: SocketConfig,
        *,
        engineio_path: str | None = "",
        fastapi_app: FastAPI,
        on_connect: Callable[[str, dict, dict | None], Awaitable[None]] = None,
        on_disconnect: Callable[[str], Awaitable[None]] = None,
    ) -> "RBSocketIO":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls(
                    conf,
                    engineio_path=engineio_path,
                    fastapi_app=fastapi_app,
                    on_connect=on_connect,
                    on_disconnect=on_disconnect,
                )
        return cls._instance

    @classmethod
    def get(cls) -> "RBSocketIO":
        inst = cls._instance
        if inst is None:
            raise RuntimeError("RBSocketIO is not initialized. Call RBSocketIO.init(...) first.")
        return inst

    async def emit(
        self,
        event: str,
        payload: dict,
        *,
        room: str | None = None,
        to: str | None = None,
        namespace: str | None = None,
    ):
        await self.sio.emit(
            event, payload, room=room, to=to, namespace=namespace or self.conf.namespace
        )

    async def join(self, sid: str, room: str, *, namespace: str | None = None):
        await self.sio.enter_room(sid, room, namespace=namespace or self.conf.namespace)

    async def leave(self, sid: str, room: str, *, namespace: str | None = None):
        await self.sio.leave_room(sid, room, namespace=namespace or self.conf.namespace)
