import json
from collections.abc import Mapping
from typing import Any

try:
    import numpy as np
except Exception:
    np = None


def snake_to_camel(s: str) -> str:
    parts = s.split("_")
    return "".join(p[:1].upper() + p[1:] for p in parts if p)


def camel_to_snake(s: str) -> str:
    return "".join("_" + word.lower() if word.isupper() else word for word in s).lstrip("_")


def t_to_dict(obj: Any) -> Any:
    # 원시형
    if obj is None or isinstance(obj, bool | int | float | str):
        return obj
    # numpy 배열
    if np is not None and isinstance(obj, np.ndarray):
        return obj.tolist()
    # bytes류
    if isinstance(obj, bytes | bytearray | memoryview):
        return list(obj)
    # dict류
    if isinstance(obj, Mapping):
        return {k: t_to_dict(v) for k, v in obj.items()}
    # 시퀀스
    if isinstance(obj, list | tuple | set):
        return [t_to_dict(v) for v in obj]
    # 일반/T-object(예: State_CoreT, N_JOINT_fT)
    if hasattr(obj, "__dict__"):
        return {k: t_to_dict(v) for k, v in vars(obj).items()}
    # 마지막 안전핀
    return str(obj)


def to_json(obj: Any) -> str:
    return json.dumps(t_to_dict(obj), ensure_ascii=False)
