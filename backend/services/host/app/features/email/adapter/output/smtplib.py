"""
[Email SMTP 라이브러리 어댑터]
"""
import smtplib
import ssl
import time
from email.message import EmailMessage

from rb_modules.log import rb_log

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

    def send_email(self, model: EmailModel) -> None: # pylint: disable=no-self-use, invalid-overridden-method
        try:
            start_time = time.perf_counter() # 시작 시간
            # 1) 이메일 메시지 생성
            msg = EmailMessage()
            password = None
            if model.from_email is None or model.from_email == "":
                msg["From"] = "rainbow.mobilerobot@gmail.com"
                password = "keyryjnriyzgoazg"
            else:
                msg["From"] = model.from_email
                password = self.password

            # list[str] 대비
            if isinstance(model.to_email, list | tuple):
                msg["To"] = ", ".join(model.to_email)
            else:
                msg["To"] = str(model.to_email)

            msg["Subject"] = model.subject
            msg.set_content(model.body if model.body is not None else "", subtype="html")

            # 2) 첨부파일 추가
            if(model.attachments is not None):
                for filename, content, mime_type in model.attachments:
                    if filename is None or content is None or mime_type is None:
                        continue
                    maintype, subtype = mime_type.split("/", 1)
                    rb_log.debug(f"filename: {filename}, mime_type: {mime_type}, maintype: {maintype}, \
                        subtype: {subtype}")
                    msg.add_attachment(content, maintype=maintype, \
                        subtype=subtype, filename=filename)

            # 3) SMTP 서버 연결 및 이메일 전송
            rb_log.info(f"📨 이메일 전송 시작 : {msg['From']} -> {msg['To']}")
            context = ssl.create_default_context()
            with smtplib.SMTP("smtp.gmail.com", 587, timeout=200) as server:
                server.ehlo()
                server.starttls(context=context)
                server.login(msg["From"], password)
                server.send_message(msg)
                end_time = time.perf_counter() # 종료 시간
                rb_log.info(f"📨 이메일 전송 완료 : {msg['From']} -> {msg['To']} (소요시간: {end_time - start_time}초)")
        except Exception as e: # pylint: disable=broad-exception-caught
            rb_log.error(f"📨 이메일 전송 실패 : {msg['From']} -> {msg['To']} (오류: {e})")
            end_time = time.perf_counter() # 종료 시간
