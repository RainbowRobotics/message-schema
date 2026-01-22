import json
import struct
from typing import Any

_HDR = 4
_MAX = 10_000_000

def pack(obj: dict[str, Any]) -> bytes:
    b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    return struct.pack(">I", len(b)) + b

async def read(reader) -> dict[str, Any]:
    h = await reader.readexactly(_HDR)
    (n,) = struct.unpack(">I", h)
    if n <= 0 or n > _MAX:
        raise ValueError(f"invalid length: {n}")
    b = await reader.readexactly(n)
    return json.loads(b.decode("utf-8"))
