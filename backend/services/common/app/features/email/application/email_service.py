"""
[Email 서비스]
"""
import asyncio
import smtplib
from email.message import EmailMessage

from app.features.email.adapter.output.smtplib import (
    EmailSmtpLibEmailAdapter,
)
from app.features.email.domain.email import (
    EmailModel,
)
from app.features.email.dto.email_dto import (
    ResponseSendEmailDto,
)


class EmailService:
    """
    [Email 서비스]
    """
    def __init__(self):
        self.email_port = EmailSmtpLibEmailAdapter(password="keyryjnriyzgoazg")

    def send_mail_sync(
        self,
        from_email: str,
        to_emails: list[str],
        subject: str,
        body: str,
        attachments: list[tuple[str, bytes, str]] | None = None,
    ) -> ResponseSendEmailDto | None:
        """실제 SMTP로 메일 보내는 동기 함수"""
        msg = EmailMessage()
        msg["From"] = from_email
        msg["To"] = ", ".join(to_emails)
        msg["Subject"] = subject

        # 단순 text라면 set_content, HTML이면 subtype 지정
        msg.set_content(body, subtype="html")  # text만 쓸 거면 subtype 빼도 됨

        for filename, content, mime_type in attachments or []:
            maintype, subtype = mime_type.split("/", 1)
            msg.add_attachment(
                content,
                maintype=maintype,
                subtype=subtype,
                filename=filename,
            )

        smtp_host = "smtp.gmail.com"
        smtp_port = 587
        smtp_user = "rainbow.mobilerobot@gmail.com"
        smtp_pass = "keyryjnriyzgoazg"

        with smtplib.SMTP(smtp_host, smtp_port) as server:
            server.starttls()
            if smtp_user and smtp_pass:
                server.login(smtp_user, smtp_pass)
            server.send_message(msg)

    async def send_email(
        self,
        to_email: list[str],
        subject: str,
        body: str,
        from_email: str | None = None,
        password: str | None = None,
        attachments: list[tuple[str, bytes, str]] | None = None,
    ) -> ResponseSendEmailDto:
        """
        - from_email: 메일 발신자(기본은 rainbow.mobilerobot@gmail.com)
        - to_email: 메일 수신자(리스트 형식)
        - subject: 메일 제목
        - body: 메일 본문
        - attachments: 첨부파일(리스트 형식)
        - return: ResponseSendEmailDto
        """

        # 0) 발신자 이메일 주소 유효성 체크

        # 1) 모델 생성
        model = EmailModel(
            from_email=from_email,
            password=password,
            to_email=to_email,
            subject=subject,
            body=body,
            attachments=attachments,
        )

        # 2) 변수 검사
        model.check_variables()

        # 3) 이메일 전송
        asyncio.create_task(
            asyncio.to_thread(
                self.email_port.send_email,
                model
            )
        )

        return ResponseSendEmailDto(
            from_email=from_email,
            to_email=to_email,
            subject=subject,
            body=body,
            attachments=[attachment[0] for attachment in attachments],
        )
