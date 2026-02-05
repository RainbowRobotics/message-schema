"""
[AMR 이동 이메일 아웃풋 포트]
"""
from typing import (
    Protocol,
    runtime_checkable,
)


@runtime_checkable
class EmailPort(Protocol):
    """
    [AMR 이동 이메일 아웃풋 포트]
    """
    async def send_export_email(self, email: str, file_path: str, file_name: str) -> None: ...
