"""
[AMR 이동 API 어댑터]
"""

from fastapi import APIRouter, BackgroundTasks
from rb_modules.log import rb_log

from app.features.move.schema.move_api import (
    Request_Move_CircularPD,
    Request_Move_GoalPD,
    Request_Move_RotatePD,
    Request_Move_TargetPD,
    Request_Move_XLinearPD,
    RequestAmrMoveArchiveLogPD,
    RequestAmrMoveExportLogPD,
    RequestAmrMoveLogsPD,
    Response_Move_CircularPD,
    Response_Move_GoalPD,
    Response_Move_LogsPD,
    Response_Move_PausePD,
    Response_Move_ResumePD,
    Response_Move_RotatePD,
    Response_Move_StopPD,
    Response_Move_TargetPD,
    Response_Move_XLinearPD,
)
from app.features.move.src.application.amr_move_service import AmrMoveService
from app.socket.socket_client import (
    socket_client,
)

# from app.main import amr_move_service
amr_move_router = APIRouter(
    tags=["AMR 이동"],
    prefix="/slamnav/move",
)

amr_move_service = AmrMoveService()


@amr_move_router.post(
    "/goal",
    summary="목표 지점으로 이동",
    description= """
SLAMNAV로 목표 노드로 주행 명령을 전달합니다.

## 📌 기능 설명
- 지도상의 목표 노드를 입력으로 받아 주행합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| goalId | string | - | - | 지도 상의 목표 노드의 ID를 입력하세요. | 'N_123' |
| preset | number | - | - | 지정된 속도프리셋을 설정합니다. | 0 |
| method | string | - | - | 주행방식을 선언합니다. 기본 pp (point to point) 방식으로 주행하며 그 외 주행방식은 모델마다 지원하는 방식이 다릅니다. | 'pp', 'hpp' |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| command | string | 요청 명령 | 'goal' |
| goalId | string | command가 goal인 경우, 목표 노드의 ID. | 'N_123' |
| preset | number | 지정된 속도프리셋 | 0 |
| method | string | 주행방식 | 'pp', 'hpp' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="이동 명령 처리 결과 반환"
)
async def slamnav_move_goal(request: Request_Move_GoalPD) -> Response_Move_GoalPD:
    return await amr_move_service.move_goal(request)

@amr_move_router.post(
    "/target",
    summary="타겟 좌표로 이동",
    description="""
SLAMNAV로 조이스틱 이동 명령을 전달합니다.

## 📌 기능 설명
- 로봇의 속도(vx, vy, wz)를 입력으로 받아 이동합니다.
- 주기적으로 계속해서 요청을 주지 않으면 주행이 중단됩니다.
- 응답 없이 일방적으로 송신합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| vx | number | - | m/s | 로봇의 x방향 속도를 입력하세요. | 1.3 |
| vy | number | - | m/s | 로봇의 y방향 속도를 입력하세요. | 1.3 |
| wz | number | - | deg/s | 로봇의 z축 회전 속도를 입력하세요. | 1.3 |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="이동 명령 처리 결과 반환"
)
async def slamnav_move_target(request: Request_Move_TargetPD) -> Response_Move_TargetPD:
    return await amr_move_service.move_target(request)

# @amr_move_router.post(
#     "/jog",
#     summary="조이스틱 이동",
#     description="""
#     AMR을 주어진 속도값으로 조이스틱 이동시킵니다. 반환값은 없습니다

#     - vx: 직진 속도 (m/s)
#     - vy: 직진 속도 (m/s)
#     - wz: 회전 속도 (deg/s)
#     """,
# )
# async def slamnav_move_jog(request: Request_Move_JogPD) -> None:
#     """
#     - request: Request_Move_JogPD
#     - amr_move_service.move_jog: amr_move_service.move_jog 메서드 호출
#     """
#     return await amr_move_service.move_jog(request)

@amr_move_router.post(
    "/stop",
    summary="이동 중지",
    description="""
SLAMNAV로 이동 정지 명령을 전달합니다.

## 📌 기능 설명
- 로봇의 주행을 중단합니다.
- 주행 중이 아닐때는 아무런 동작을 하지 않습니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| command | string | 요청 명령 | 'stop' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="이동 중지 명령 처리 결과 반환"
)
async def slamnav_move_stop() -> Response_Move_StopPD:
    return await amr_move_service.move_stop()


@amr_move_router.post(
    "/pause",
    summary="이동 일시정지",
    description="""
SLAMNAV로 이동 일시정지 명령을 전달합니다.

## 📌 기능 설명
- 로봇의 주행을 일시정지합니다.
- 주행 중이 아닐때는 아무런 동작을 하지 않습니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| command | string | 요청 명령 | 'pause' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="이동 일시정지 명령 처리 결과 반환"
)
async def slamnav_move_pause() -> Response_Move_PausePD:
    return await amr_move_service.move_pause()

@amr_move_router.post(
    "/resume",
    summary="이동 재개",
    description="""
SLAMNAV로 이동 일시재개 명령을 전달합니다.

## 📌 기능 설명
- 로봇의 주행을 일시재개합니다.
- 주행 일시정지 상태에서만 일시재개할 수 있습니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| command | string | 요청 명령 | 'resume' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="이동 재개 명령 처리 결과 반환"
)
async def slamnav_move_resume() -> Response_Move_ResumePD:
    return await amr_move_service.move_resume()


@amr_move_router.post(
    "/linear",
    summary="선형 이동 명령",
    description="""
SLAMNAV로 선형 이동 명령을 전달합니다.

## 📌 기능 설명
- 자율주행이 아닌 일정 거리만큼 직진주행합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| target | number | ✅ | m | 목표 위치를 입력하세요. | 1.3 |
| speed | number | ✅ | m/s | 주행 속도를 입력하세요. | 1.3 |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| target | number | - | m | 목표 위치를 입력하세요. | 1.3 |
| speed | number | - | m/s | 주행 속도를 입력하세요. | 1.3 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="프로필 이동 명령 처리 결과 반환"
  )
async def slamnav_move_linear(request: Request_Move_XLinearPD) -> Response_Move_XLinearPD:
    return await amr_move_service.move_linear(request)


@amr_move_router.post(
    "/circular",
    summary="회전 주행 명령",
    description="""
SLAMNAV로 회전 명령을 전달합니다.

## 📌 기능 설명
- 자율주행이 아닌, 로봇의 z축을 기준으로 회전합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| target | number | ✅ | deg | 목표 위치를 입력하세요. | 1.3 |
| speed | number | ✅ | deg/s | 주행 속도를 입력하세요. | 1.3 |
| direction | string | - | - | command가 circular인 경우, 주행 방향을 입력하세요. | 'left', 'right' |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| target | number | - | m | 목표 위치를 입력하세요. | 1.3 |
| speed | number | - | m/s | 주행 속도를 입력하세요. | 1.3 |
| direction | string | - | - | 주행 방향을 입력하세요. | 'left', 'right' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="프로필 이동 명령 처리 결과 반환"
  )
async def slamnav_move_circular(request: Request_Move_CircularPD) -> Response_Move_CircularPD:
    return await amr_move_service.move_circular(request)

@amr_move_router.post(
    "/rotate",
    summary="회전 명령",
    description="""
SLAMNAV로 회전 명령을 전달합니다.

## 📌 기능 설명
- 자율주행이 아닌, 로봇의 z축을 기준으로 회전합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| target | number | ✅ | deg | 목표 회전 각도를 입력하세요. | 30 |
| speed | number | ✅ | deg/s | 회전 속도를 입력하세요. | 10 |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| target | number | - | deg | 목표 회전 각도를 입력하세요. | 30 |
| speed | number | - | deg/s | 회전 속도를 입력하세요. | 10 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="프로필 이동 명령 처리 결과 반환"
  )
async def slamnav_move_rotate(request: Request_Move_RotatePD) -> Response_Move_RotatePD:
    return await amr_move_service.move_rotate(request)

@amr_move_router.post(
    "/logs",
    summary="이동 로그 조회",
    description="""
    이동 로그를 조회합니다.

    - limit: 페이지 당 로그 수
    - page: 페이지 번호
    - searchText: 검색어. 텍스트 필드에서 일치하는 내용을 검색합니다.
    - sort: 정렬 기준 필드
    - order: 정렬 순서 "asc" 또는 "desc"
    - filter: 검색 필터. JSON 문자열로 입력합니다. MongoDB 조건식에 맞춰 입력해주세요. 예) {'result': 'success'}
    - fields: 반환 필드. 입력이 없으면 저장된 모든 필드가 반환되며 입력한 필드의 값에 따라 특정 필드만 반환받거나 특정 필드를 제외하고 반환받고 싶을때 사용하세요. 예) id필드만 제외하고 반환 {'id': 0}, id와 status필드만 반환 {'id': 1, 'status': 1}
    """,
    response_description="이동 로그 조회 결과 반환"
)
async def slamnav_move_logs(
    request: RequestAmrMoveLogsPD) -> Response_Move_LogsPD:
    """
    - request: RequestAmrMoveLogsPD
    - amr_move_service.get_logs: amr_move_service.get_logs 메서드 호출
    - 이동 로그 조회 결과 반환
    """
    return await amr_move_service.get_logs(request)


@amr_move_router.delete(
    "/logs",
    summary="이동 로그 아카이브(디버그)",
    description="이동 로그를 삭제합니다. 입력된 기간동안의 로그를 삭제하며 옵션으로는 삭제할 로그를 압축보관할 지 여부를 결정할 수 있습니다",
    response_description="이동 로그 삭제 결과 반환"
)
async def archive_move_logs(request: RequestAmrMoveArchiveLogPD):
    """
    - request: RequestAmrMoveArchiveLogPD
    - amr_move_service.archive_logs: amr_move_service.archive_logs 메서드 호출
    - 이동 로그 아카이브 결과 반환
    """
    return await amr_move_service.archive_logs(request)


@amr_move_router.post(
    "/export",
    summary="이동 로그 내보내기",
    description="""
    검색과 동시에 이동 로그를 파일 혹은 메일을 이용해 내보냅니다.
    - startDt: 내보내기 기준 시작 날짜 (해당날짜기준부터 내보냅니다)
    - endDt: 내보내기 기준 종료 날짜 (해당날짜기준까지 내보냅니다)
    - filters: 검색 필터. JSON 문자열로 입력합니다. MongoDB 조건식에 맞춰 입력해주세요. 예) {'result': 'success'}
    - searchText: 검색어. 텍스트 필드에서 일치하는 내용을 검색합니다.
    - sort: 정렬 기준 필드
    - order: 정렬 순서 "asc" 또는 "desc"
    - method: 내보내기 방식 (file, email)
    - filename: 내보내기 파일명(확장자는 gz로 고정됩니다)
    - email: 메일 주소 (method가 email인 경우 필수)
    """,
    response_description="이동 로그 내보내기 결과 반환"
)
async def export_move_logs(dto: RequestAmrMoveExportLogPD, background_tasks: BackgroundTasks):
    """
    - request: RequestAmrMoveExportLogPD
    - amr_move_service.export_logs: amr_move_service.export_logs 메서드 호출
    - 이동 로그 내보내기 결과 반환
    """
    return await amr_move_service.export_logs(dto, background_tasks)




@amr_move_router.get("/test")
async def test_move():
    rb_log.info("============== 테스트 =================")
    await socket_client.emit("test/v1/move/test", "test")
