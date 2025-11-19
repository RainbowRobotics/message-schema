"""
[Email 모델]
"""
from dataclasses import dataclass
from typing import Any

from pydantic import BaseModel


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
    from_email: str = "ra inbow.mobilerobot@gmail.com"
    to_email: list[str] | None = None
    subject: str | None = None
    body: str | None = None
    attachments: list[EmailAttachment] | None = None

    def __init__(self, to_email: list[str], subject: str, from_email: str | None = None, body: str | None = None, attachments: list[tuple[str, bytes, str]] | None = None):
        self.from_email = from_email if from_email is not None else self.from_email
        self.to_email = to_email
        self.subject = subject
        self.body = body

        for filename, content, mime_type in attachments:
            if filename is None or content is None or mime_type is None:
                continue
            print(f"filename: {filename}, content: {content}, mime_type: {mime_type}")
            maintype, subtype = mime_type.split("/", 1)
            self.attachments.append(EmailAttachment(content=content, maintype=maintype, subtype=subtype, filename=filename))

    def check_variables(self) -> None:
        """
        - return: None
        """
        if self.from_email is None:
            raise ValueError("from_email is required")
        if self.to_email is None:
            raise ValueError("to_email is required")
        if self.subject is None:
            raise ValueError("subject is required")
        if self.body is None:
            raise ValueError("body is required")
        if self.attachments is None:
            raise ValueError("attachments is required")

    def to_dict(self) -> dict[str, Any]:
        """
        - return: dict[str, Any]
        """
        return self.__dict__
