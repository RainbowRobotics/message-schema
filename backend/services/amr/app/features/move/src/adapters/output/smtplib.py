from app.features.move.src.port.email_output import (
    MoveEmailPort,
)
import os, ssl, smtplib
from email.message import EmailMessage

class MoveSmtpLibEmailAdapter(MoveEmailPort):
    """
    [AMR 이동 이메일 아웃풋 포트]
    """
    async def send_export_email(self, email: str, file_path: str, file_name: str) -> None:
        msg = EmailMessage()
        msg["From"] = "rainbow.mobilerobot@gmail.com"
        msg["To"] = email
        msg["Subject"] = "[RRS] 이동 로그 내보내기 결과"
        msg.set_content("요청하신 이동 로그 파일을 첨부합니다.\n\n- 파일: {0}".format(file_name))

        with open(file_path, "rb") as f:
            msg.add_attachment(
                f.read(),
                maintype="application",
                subtype="gzip",
                filename=file_name,
            )

        # 1) 587 + STARTTLS
        context = ssl.create_default_context()
        with smtplib.SMTP("smtp.gmail.com", 587, timeout=20) as server:
            server.ehlo()
            server.starttls(context=context)
            server.login("rainbow.mobilerobot@gmail.com", "keyryjnriyzgoazg")
            server.send_message(msg)
