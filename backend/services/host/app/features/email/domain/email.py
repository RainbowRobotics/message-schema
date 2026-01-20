"""
[Email 모델]
"""
from dataclasses import dataclass
from typing import Any

from pydantic import BaseModel
from rb_utils.service_exception import (
    ServiceException,  # pylint: disable=import-error,no-name-in-module
)


class EmailAttachment(BaseModel):
    """
    [Email 첨부파일]
    """
    content: bytes
    maintype: str
    subtype: str
    filename: str

@dataclass
class EmailModel:
    """
    [Email 모델]
    """
    from_email: str | None = None
    password: str | None = None
    to_email: list[str] | None = None
    subject: str | None = None
    body: str | None = None
    attachments: list[tuple[str, bytes, str]] | None = None

    def __init__(self, to_email: list[str], subject: str, from_email: str | None = None, password: str | None = None, body: str | None = None, attachments: list[tuple[str, bytes, str]] | None = None):
        self.from_email = from_email if from_email is not None else self.from_email
        self.password = password if password is not None else self.password
        self.to_email = to_email
        self.subject = subject
        self.body = body
        self.attachments = attachments
        # print(f"attachments: {len(attachments) if attachments else 0}")

        # for filename, content, mime_type in attachments or []:
        #     if filename is None or content is None or mime_type is None:
        #         continue
        #     maintype, subtype = mime_type.split("/", 1)
        #     print(f"filename: {filename}, mime_type: {mime_type}, maintype: {maintype}, subtype: {subtype}")
        #     self.attachments.append({content=content, maintype=maintype, subtype=subtype, filename=filename})

    def check_variables(self) -> None:
        """
        - return: None
        """
        if self.to_email is None:
            raise ServiceException("to_email 값이 없습니다.", 403)
        if self.subject is None:
            raise ServiceException("subject 값이 없습니다.", 403)
        if self.body is None:
            raise ServiceException("body 값이 없습니다.", 403)

    def to_dict(self) -> dict[str, Any]:
        """
        - return: dict[str, Any]
        """
        return self.__dict__
