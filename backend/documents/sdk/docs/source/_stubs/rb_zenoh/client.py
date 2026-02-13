from typing import Any, Protocol, TypeVar


T = TypeVar("T")


class FBRootReadable(Protocol[T]):
    @staticmethod
    def InitFromPackedBuf(buf: bytes, pos: int = 0) -> T: ...


class ZenohClient:
    def __init__(self, *args, **kwargs):
        pass
