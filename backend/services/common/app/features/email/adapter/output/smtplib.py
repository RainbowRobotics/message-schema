"""
[Email SMTP 라이브러리 어댑터]
"""
import ssl, smtplib
from app.features.email.port.email_send_port import (
    EmailSendPort,
)
from app.features.email.domain.email import (
    EmailModel,
)
from email.message import EmailMessage

class EmailSmtpLibEmailAdapter(EmailSendPort):
    """
    [Email SMTP 라이브러리 어댑터]
    """
    def __init__(self, password: str):
        self.password = password

    async def send_email(self, model: EmailModel) -> None:
        msg = EmailMessage()
        msg["From"] = model.from_email
        msg["To"] = model.to_email
        msg["Subject"] = model.subject
        msg.set_content(model.body if model.body is not None else "")

        if model.attachment is not None:
            for attachment in model.attachment:
                with open(attachment.file_path, "rb") as f:
                    msg.add_attachment(
                        f.read(),
                        maintype="application",
                        subtype=attachment.file_type,
                        filename=attachment.file_name,
                    )

        # 1) 587 + STARTTLS
        context = ssl.create_default_context()
        with smtplib.SMTP("smtp.gmail.com", 587, timeout=20) as server:
            server.ehlo()
            server.starttls(context=context)
            server.login(model.from_email, self.password)
            server.send_message(msg)
