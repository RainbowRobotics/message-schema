"""
[서비스 예외 클래스]
"""
from __future__ import (
    annotations,
)

from dataclasses import (
    dataclass,
)
from typing import Any


@dataclass(slots=True)
class ServiceException(Exception):
    """
    - message: 예외 메시지
    - status_code: HTTP 상태 코드 (기본: None)
    """
    message: str
    status_code: int | None = None   # HTTP 등

    def __str__(self) -> str:
        base = self.message
        if self.status_code is not None:
            base += f" (status={self.status_code})"
        return base

    def to_dict(self) -> dict[str, Any]:
        """
        - message: 예외 메시지
        - status_code: HTTP 상태 코드
        """
        return {
            "message": self.message,
            "status_code": self.status_code,
        }
