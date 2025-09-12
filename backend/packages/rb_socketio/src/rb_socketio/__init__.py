from .config import SocketConfig
from .core import RBSocketIO
from .ns_client import RBSocketIONsClient
from .socket_client_router import RbSocketIORouter
from .subscriber import RBSocketIOSubscriber
from .subscriber_manager import RBSocketIOSubscriberManager

# rb_sio = RBSocketIO.get().sio


def get_rb_sio():
    return RBSocketIO.get().sio


__all__ = [
    "SocketConfig",
    "RBSocketIOSubscriber",
    "RBSocketIOSubscriberManager",
    "RBSocketIO",
    "RBSocketIONsClient",
    "RbSocketIORouter",
    "get_rb_sio",
]
