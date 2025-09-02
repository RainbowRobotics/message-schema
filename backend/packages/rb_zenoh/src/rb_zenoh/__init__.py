from collections.abc import Callable

from .main import ZenohClient
from .router import ZenohRouter
from .schema import SubscribeOptions

zenoh_router = ZenohRouter()

zenoh_client = ZenohClient()


def zenoh_subscribe(topic: str, opts: SubscribeOptions | None = None) -> Callable:
    """
    예:
        @zenoh_subscribe("example/demo")
        def on_demo(topic, mv, obj):
            ...
    """

    def decorator(func: Callable):

        zenoh_client.subscribe(topic, func, opts or SubscribeOptions())
        return func

    return decorator
