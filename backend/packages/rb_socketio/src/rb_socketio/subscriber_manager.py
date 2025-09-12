import importlib
import pkgutil

import socketio

from .subscriber import RBSocketIOSubscriber


class RBSocketIOSubscriberManager:
    def __init__(self, sio: socketio.AsyncServer, default_namespace: str):
        self.sio = sio
        self.default_namespace = default_namespace

    def register(self, *routers: RBSocketIOSubscriber):
        for r in routers:
            ns = r.namespace or self.default_namespace
            r._register_to(self.sio, ns)
        return self

    def scan(self, package_path: str, attr: str | None = None):
        pkg = importlib.import_module(package_path)
        for m in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + "."):
            mod = importlib.import_module(m.name)

            if attr and hasattr(mod, attr):
                obj = getattr(mod, attr)
                if isinstance(obj, RBSocketIOSubscriber):
                    self.register(obj)
                elif isinstance(obj, list | tuple):
                    found = [x for x in obj if isinstance(x, RBSocketIOSubscriber)]
                    if found:
                        self.register(*found)
                continue

            for _, val in vars(mod).items():
                if getattr(val, "__is_sio_subscriber__", False):
                    self.register(val)
        return self
