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
    file_path: str
    file_name: str
    file_content: bytes
    file_type: str

@dataclass
class EmailModel:
    """
    [Email 모델]
    """
    from_email: str = "rainbow.mobilerobot@gmail.com"
    to_email: str
    subject: str
    body: str | None = None
    attachment: list[EmailAttachment] | None = None

    def __init__(self, to_email: str, subject: str, from_email: str | None = None, body: str | None = None, attachment: list[EmailAttachment] | None = None):
        self.from_email = from_email if from_email is not None else self.from_email
        self.to_email = to_email
        self.subject = subject
        self.body = body
        self.attachment = attachment

    def to_dict(self) -> dict[str, Any]:
        """
        - return: dict[str, Any]
        """
        return self.__dict__
