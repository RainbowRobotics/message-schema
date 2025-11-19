"""
[Email SMTP 라이브러리 어댑터]
"""
import smtplib
import ssl
from email.message import EmailMessage

from app.features.email.domain.email import (
    EmailModel,
)
from app.features.email.port.email_send_port import (
    EmailSendPort,
)


class EmailSmtpLibEmailAdapter(EmailSendPort):
    """
    [Email SMTP 라이브러리 어댑터]
    """
    def __init__(self, password: str):
        self.password = password

    async def send_email(self, model: EmailModel) -> None:
        # 1) 이메일 메시지 생성
        msg = EmailMessage()
        msg["From"] = model.from_email
        msg["To"] = model.to_email
        msg["Subject"] = model.subject
        msg.set_content(model.body if model.body is not None else "", subtype="html")

        # 2) 첨부파일 추가
        for attachment in model.attachments:
            msg.add_attachment(attachment)

        # 3) SMTP 서버 연결 및 이메일 전송
        context = ssl.create_default_context()
        with smtplib.SMTP("smtp.gmail.com", 587, timeout=20) as server:
            server.ehlo()
            server.starttls(context=context)
            server.login(model.from_email, self.password)
            server.send_message(msg)
