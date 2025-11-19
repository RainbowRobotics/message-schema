"""
[Email API]
"""

from typing import Annotated

from fastapi import APIRouter, File, Form, UploadFile
from rb_modules.log import RBLog

from app.features.email.application.email_service import EmailService

email_router = APIRouter(
    tags=["Email"],
    prefix="/email"
)
email_service = EmailService()
rb_log = RBLog()

@email_router.post("/send")
async def send_email(
    from_email: Annotated[str, Form(...)],
    to_email: Annotated[list[str], Form(...)],
    subject: Annotated[str, Form(...)],
    body: Annotated[str, Form(...)],
    attachments: File()
    ):
    """
    [Email 전송]
    - from_email: str (Form)
    - to_email: list[str] (Form, 여러 번)
    - subject: str (Form)
    - body: str (Form)
    - attachments: List[UploadFile] (File, 여러 개)
    """
    rb_log.info(f"[send_email] Email 전송 요청: from_email={from_email}, to_email={to_email}, subject={subject}, body={body}, attachmentsSize={len(attachments) if attachments else 0}")

    # 1) 첨부파일 가공: UploadFile -> (filename, bytes, mimetype)
    files_data: list[tuple[str, bytes, str]] = []
    if attachments:
        for f in attachments:
            content = await f.read()
            files_data.append((f.filename, content, f.content_type or "application/octet-stream"))

    # 2) 서비스 호출
    return await email_service.send_email(from_email, to_email, subject, body, attachments=files_data if attachments else [])
