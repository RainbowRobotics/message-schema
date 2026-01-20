"""
[Email API]
"""


from typing import Annotated

from email_validator import validate_email
from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from rb_modules.log import RBLog  # pylint: disable=import-error,no-name-in-module

from app.services.host.email.application.email_service import EmailService

email_router = APIRouter(
    tags=["Email"],
    prefix="/email"
)
email_service = EmailService()
rb_log = RBLog()
SMTP_MAX_SIZE = 20 * 1024 * 1024

def validate_mx(email: str) -> bool:
    """
    - return: bool
    """
    try:
        validate_email(email)
        return True
    except Exception as e: # pylint: disable=broad-exception-caught
        rb_log.error(f"이메일 주소 유효성 체크 예외 발생: {e}")
        return False

@email_router.post("/send")
async def send_email(
    to_email: Annotated[list[str], Form(..., example=["blike1234@gmail.com"])],
    subject: str = Form(..., example="안녕 나는 스웨거테스트"),
    body: str = Form(..., example="안녕 나는 스웨거테스트 본문"),
    from_email: Annotated[str | None, Form()] = None,
    password: Annotated[str | None, Form()] = None,
    attachments: Annotated[list[UploadFile] | None, File()] = None,
    ):
    '''
    # email 전송 요청

    ## 🧩 기능 설명
     - 이메일을 전송합니다.
     - 첨부파일은 총 20MB를 초과할 수 없으며 여러개의 파일을 첨부할 수 있습니다.
     - 수신자메일은 한꺼번에 여러명에게 전송할 수 있습니다.
     - 메일 전송 요청 후 응답합니다. 메일 전송 성공 여부는 포함하지 않습니다.
     - ***메일 발신자 이메일 주소는 기본 rainbow.mobilerobot@gmail.com 입니다.***
     - 별도의 발신자 이메일 주소로 송신하려면 from_email과 password 필드를 입력해야합니다.
     - 메일 전송 요청이 성공으로 떠도 아래와 같은 이유로 메일 전송이 실패할 수 있습니다.
       - 발신자 이메일주소와 패스워드가 유효하지 않습니다.
       - 전송 제한 시간(200초) 초과로 인한 예외 발생
       - 수신자 이메일 주소가 유효하지 않습니다.
       - 서버 오류로 인한 쓰레드 종료

    ## 🏷️ 요청 바디 (multipart/form-data)
    | 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
    |-|-|-|-|-|-|
    | to_email | list[str] | ✅ | - | 메일 수신자 | ["test@gmail.com", "test2@gmail.com"] |
    | subject | str | ✅ | - | 메일 제목 | "test" |
    | body | str | ✅ | - | 메일 본문 | "test" |
    | from_email | str | - | - | 메일 발신자. 입력이 없으면 기본 이메일로 발송 |  |
    | password | str | - | - | SMTP 발신자 비밀번호 |  |
    | attachments | List[file] | - | - | 첨부파일 | [] |

    ## 📝 응답 바디
    | 필드명 | 타입 | 설명 | 예시 |
    |-----|-----|-----|-----|
    | from_email | str | 메일 발신자 | rainbow.mobilerobot@gmail.com |
    | to_email | list[str] | 메일 수신자 | ["test@gmail.com", "test2@gmail.com"] |
    | subject | str | 메일 제목 | "test" |
    | body | str | 메일 본문 | "test" |
    | attachments | list[str] | 첨부파일 이름목록 | ["test.pdf"] |

    ## ⚠️ 에러 리스트
    ---
    ## ***422*** Unprocessable Entity
    - 파라메터가 유효하지 않습니다.
    - ***swagger 테스트 시, send empty value 체크박스는 반드시 체크 해제*** 후 보내야 합니다.
    ## ***403*** Invalid Request
    - 첨부파일 크기가 20MB를 초과합니다.
    - 발신자 이메일 주소가 유효하지 않습니다.
    - 수신자 이메일 주소가 유효하지 않습니다.
    ## ***500*** Internal Server Error
    - 서버 내부 로직 예외 발생

    ***Swagger 테스트 시, Send empty value 체크박스는 반드시 체크 해제*** 후 보내야 합니다.
    '''
    rb_log.info(f"[send_email] Email 전송 요청: from_email={from_email}, to_email={to_email}, \
        subject={subject}, body={body}, \
            attachments={len(attachments) if attachments else 0}")

    # 1) 이메일 주소 유효성 체크
    if from_email is not None and from_email != "":
        if not validate_mx(from_email):
            raise HTTPException(status_code=403, detail=f"발신자 이메일 주소가 유효하지 않습니다. {from_email}")
        if password is None or password == "":
            raise HTTPException(status_code=403, detail="SMTP 발신자 비밀번호가 유효하지 않습니다.")

    for email in to_email:
        if not validate_mx(email):
            raise HTTPException(status_code=403, detail=f"수신자 이메일 주소가 유효하지 않습니다. {email}")

    # 2) 첨부파일 가공: UploadFile -> (filename, bytes, mimetype)
    files_data: list[tuple[str, bytes, str]] = []
    if attachments:
        for f in attachments:
            content = await f.read()
            files_data.append((f.filename, content, f.content_type or "application/octet-stream"))

    # 3) 첨부파일 크기 체크 후 에러
    total_size = sum(len(content) for _, content, _ in files_data)

    if total_size > SMTP_MAX_SIZE:
        raise HTTPException(status_code=403, detail="첨부파일 크기가 20MB를 초과합니다.")

    # 4) 서비스 호출
    return await email_service.send_email(to_email, subject, body, \
        from_email, password, \
        attachments=files_data if attachments else [])
