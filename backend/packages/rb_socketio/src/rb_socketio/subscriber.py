from collections.abc import Callable
from typing import Any

import socketio


class RBSocketIOSubscriber:
    def __init__(self, prefix: str = "", namespace: str | None = None):
        self.prefix = prefix.strip("/")
        self.namespace = namespace
        self._handlers: list[tuple[str, Callable[..., Any]]] = []
        self._connect: Callable[..., Any] | None = None
        self._disconnect: Callable[..., Any] | None = None
        self.__is_sio_subscriber__ = True

    def on(self, event: str):
        def deco(fn: Callable[..., Any]):
            full = f"{self.prefix}/{event}" if self.prefix else event
            self._handlers.append((full, fn))
            return fn

        return deco

    def on_connect(self):
        def deco(fn: Callable[..., Any]):
            self._connect = fn
            return fn

        return deco

    def on_disconnect(self):
        def deco(fn: Callable[..., Any]):
            self._disconnect = fn
            return fn

        return deco

    def _register_to(self, sio: socketio.AsyncServer, namespace: str):
        if self._connect:
            sio.event(namespace=namespace)(self._connect)
        if self._disconnect:
            sio.event(namespace=namespace)(self._disconnect)
        for evt, handler in self._handlers:
            sio.on(evt, namespace=namespace)(handler)
