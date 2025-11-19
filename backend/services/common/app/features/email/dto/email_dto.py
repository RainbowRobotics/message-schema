

from fastapi import Form
from pydantic import BaseModel, Field


class RequestSendEmailDto(BaseModel):
    from_email: str = Form(..., description="메일 발신자", example="rainbow.mobilerobot@gmail.com")
    to_email: list[str] = Form(...),
    subject: str = Form(...),
    body: str = Form(...),

class ResponseSendEmailDto(BaseModel):
    success: bool = Field(..., description="메일 전송 성공 여부", example=True)
    message: str = Field(..., description="메일 전송 결과", example="메일 전송 성공")
    from_email: str = Field(..., description="메일 발신자", example="rainbow.mobilerobot@gmail.com")
    to_email: list[str] = Field(..., description="메일 수신자", example=["rainbow.mobilerobot@gmail.com"]),
    subject: str = Field(..., description="메일 제목", example="test")
    body: str = Field(..., description="메일 본문", example="test")
    attachments_size: int = Field(..., description="첨부파일 개수", example=0)
