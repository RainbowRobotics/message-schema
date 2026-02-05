import smtplib
import ssl
from email.message import EmailMessage

from app.features.localization.port.email_port import EmailPort


class LocalizationSmtpLibEmailAdapter(EmailPort):
    """
    [AMR 위치 초기화 이메일 아웃풋 포트]
    """
    async def send_export_email(self, email: str, file_path: str, file_name: str) -> None:
        msg = EmailMessage()
        msg["From"] = "rainbow.mobilerobot@gmail.com"
        msg["To"] = email
        msg["Subject"] = "[RRS] Localization 로그 내보내기 결과"
        msg.set_content(f"요청하신 Localization 로그 파일을 첨부합니다.\n\n- 파일: {file_name}")

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
