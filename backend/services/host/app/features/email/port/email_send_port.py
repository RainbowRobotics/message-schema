"""
[Email 이메일 전송 포트]
"""
from typing import (
    Protocol,
    runtime_checkable,
)

from app.features.email.domain.email import (
    EmailModel,
)


@runtime_checkable
class EmailSendPort(Protocol):
    """
    [Email 이메일 전송 포트]
    """
    async def send_email(self, model: EmailModel) -> None: ...
