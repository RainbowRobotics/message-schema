"""
[파서 유틸리티]
"""
import json
from collections.abc import (
    Mapping,
)
from datetime import datetime
from typing import Any

import numpy as np


def snake_to_camel(s: str) -> str:
    """
    - s: 스네이크 케이스 문자열
    - 스네이크 케이스 문자열을 카멜 케이스 문자열로 변환
    """
    parts = s.split("_")
    return "".join(p[:1].upper() + p[1:] for p in parts if p)


def camel_to_snake(s: str) -> str:
    """
    - s: 카멜 케이스 문자열
    - 카멜 케이스 문자열을 스네이크 케이스 문자열로 변환
    """
    return "".join("_" + word.lower() if word.isupper() else word for word in s).lstrip("_")


def t_to_dict(obj: Any) -> Any:
    """
    - obj: 객체
    - 객체를 딕셔너리로 변환
    """
    # 원시형
    if obj is None or isinstance(obj, bool | int | float | str):
        return obj
    # numpy 배열
    if np is not None and isinstance(obj, np.ndarray):
        return obj.tolist()
    # bytes류 → string 디코딩
    if isinstance(obj, bytes | bytearray | memoryview):
        if isinstance(obj, memoryview):
            obj = bytes(obj)
        try:
            return obj.decode("utf-8")
        except UnicodeDecodeError:
            return list(obj)
    # dict류
    if isinstance(obj, Mapping):
        return {k: t_to_dict(v) for k, v in obj.items()}
    # 시퀀스
    if isinstance(obj, list | tuple | set):
        return [t_to_dict(v) for v in obj]
    # 일반/T-object
    if hasattr(obj, "__dict__"):
        return {k: t_to_dict(v) for k, v in vars(obj).items()}
    # 마지막 안전핀
    return str(obj)


def _normalize_filter( filter_: Any) -> dict:
    """
    filter_가 None, dict, JSON string 모두 OK.
    빈 문자열/공백/'null'/'None'이면 {} 반환.
    """
    if filter_ is None:
        return {}
    if isinstance(filter_, Mapping):
        return dict(filter_)
    if isinstance(filter_, str):
        s = filter_.strip()
        print("s : ", s)
        if s == "" or s.lower() in ("null", "none"):
            return {}
        try:
            data = json.loads(s)
        except Exception as e:
            raise TypeError(f"filter_ JSON 파싱 실패: {e}") from e
        if not isinstance(data, dict):
            raise TypeError("filter_ 문자열은 JSON object여야 합니다.")
        return data
    raise TypeError(f"filter_ 타입이 잘못되었습니다: {type(filter_)}")

def to_json(obj: Any) -> str:
    """
    - obj: 객체
    - 객체를 JSON 문자열로 변환
    """
    return json.dumps(t_to_dict(obj), ensure_ascii=False)


def to_iso(val: str | datetime):
    """
    - val: 문자열 또는 datetime
    - 문자열 또는 datetime를 ISO 형식의 문자열로 변환
    """
    if isinstance(val, datetime):
        return val.isoformat()

    return val
