import asyncio
import socket
from typing import Any

from .parser import t_to_dict


def ManipulateZenohResHelper(obj: Any):
    dict_obj = t_to_dict(obj)

    if dict_obj.hasAttr("returnValue"):
        dict_obj.set("return_value", dict_obj.get("returnValue"))
        del dict_obj["returnValue"]

    raise ValueError("obj is not a dict")


async def get_current_ip(timeout: float = 0.5) -> str:
    hostname = socket.gethostname()
    loop = asyncio.get_running_loop()
    try:
        return await asyncio.wait_for(
            loop.run_in_executor(None, socket.gethostbyname, hostname),
            timeout=timeout,
        )
    except Exception:
        # DNS가 막히면 이전 IP를 쓰거나, 로컬 IP로 폴백
        return "127.0.0.1"
