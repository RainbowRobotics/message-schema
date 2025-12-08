


from pydantic import BaseModel, Field


class ResponseSendEmailDto(BaseModel):
    """
    [Email 전송 응답 모델]
    """
    from_email: str | None = Field(...)
    to_email: list[str] = Field(...)
    subject: str = Field(...)
    body: str = Field(...)
    attachments: list[str] = Field(...)
