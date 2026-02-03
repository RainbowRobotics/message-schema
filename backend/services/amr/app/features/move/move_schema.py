from datetime import (
    date,
    datetime,
)
from typing import (
    Any,
    Literal,
)

from pydantic import (
    BaseModel,
    Field,
)
from rb_utils.pagination import (
    PageInfo,
)


class Request_Move_GoalPD(BaseModel):
    goalId: str | None = Field(None, description="이동할 목표 지점의 ID", example="N_1734940733829")
    goalName: str | None = Field(None, description="이동할 목표 지점의 이름", example="")
    method: str = Field(..., description="이동 방식", example="pp")
    preset: int = Field(..., description="사전 설정된 이동 프로파일 번호", example=0)

class Response_Move_GoalPD(BaseModel):
    goalId: str = Field(..., description="이동한 목표 지점 ID", example="N_1734940733829")
    method: str = Field(..., description="사용된 이동 방식", example="pp")
    preset: int = Field(..., description="사용된 이동 프로파일 번호", example=0)
    result: str | None = Field(..., description="이동 명령 처리 결과", example="accept")
    message: str | None | None = Field(None, description="상태 메시지", example="")

class Request_Move_TargetPD(BaseModel):
    goalPose: list[float] = Field(..., description="이동할 목표 좌표 [x, y, z, rz]", example=[3.5,2.7,0,90])
    method: str = Field(..., description="이동 방식", example="pp")
    preset: int = Field(..., description="사전 설정된 이동 프로파일 번호", example=1)


class Response_Move_TargetPD(BaseModel):
    goalPose: list[float] = Field(..., description="이동한 목표 좌표 [x, y, z, rz]", example=[3.5,2.7,0,90])
    method: str = Field(..., description="사용된 이동 방식", example="pp")
    preset: int = Field(..., description="사용된 이동 프로파일 번호", example=0)
    result: str = Field(..., description="이동 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")


class Request_Move_JogPD(BaseModel):
    vx: float = Field(..., description="X 방향 속도 (m/s)", example=0.5)
    vy: float = Field(..., description="Y 방향 속도 (m/s)", example=0.3)
    wz: float = Field(..., description="Z축 각속도 (rad/s)", example=0.1)

class Response_Move_StopPD(BaseModel):
    result: str = Field(..., description="이동 중지 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")

class Response_Move_PausePD(BaseModel):
    result: str = Field(..., description="이동 일시정지 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")


class Response_Move_ResumePD(BaseModel):
    result: str = Field(..., description="이동 재개 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")

class Request_Move_LinearPD(BaseModel):
    target: float = Field(..., description="이동 목표 좌표 [m]", example=3.5)
    speed: float = Field(..., description="이동 속도 [m/s]", example=0.5)

class Response_Move_LinearPD(BaseModel):
    target: float = Field(..., description="이동 목표 좌표 [m]", example=3.5)
    speed: float = Field(..., description="이동 속도 [m/s]", example=0.5)
    result: str = Field(..., description="이동 선형 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")

class Request_Move_CircularPD(BaseModel):
    direction: str = Field(..., description="이동 방향 (left, right)", example="left")
    target: float = Field(..., description="이동 목표 좌표 [m, deg]", example=3.5)
    speed: float = Field(..., description="이동 속도 [m/s, deg/s]", example=0.5)

class Response_Move_CircularPD(BaseModel):
    direction: str = Field(..., description="이동 방향 (left, right)", example="left")
    target: float = Field(..., description="이동 목표 좌표 [m, deg]", example=3.5)
    speed: float = Field(..., description="이동 속도 [m/s, deg/s]", example=0.5)
    result: str = Field(..., description="이동 원형 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")

class Request_Move_RotatePD(BaseModel):
    target: float = Field(..., description="이동 목표 좌표 [deg]", example=30)
    speed: float = Field(..., description="이동 속도 [deg/s]", example=10)

class Response_Move_RotatePD(BaseModel):
    target: float = Field(..., description="이동 목표 좌표 [deg]", example=30)
    speed: float = Field(..., description="이동 속도 [deg/s]", example=10)
    result: str = Field(..., description="이동 회전 명령 처리 결과", example="accept")
    message: str | None = Field(None, description="상태 메시지", example="")

class Request_Move_LogsPD(BaseModel):
    limit: int | None = Field(None, example=10)
    page: int | None = Field(None, example=1)
    searchText: str | None = Field(None, example="")
    sort: str | None = Field(None, example="createdAt")
    order: Literal["asc", "desc"] | None = Field(None, example="desc")
    filter: dict[str, Any] | None = Field(None, example={'result': 'success'})
    fields: dict[str, Any] | None = Field(None, example={'id': 0})

# class Request_Move_LogsPD(BaseModel):
#     limit: Annotated[int, Query(10, description="페이지 당 로그 수", example=10)]
#     page: int = Query(..., description="페이지 번호", example=1)
#     searchText: str | None = Query(None, description="검색어", example="")
#     sort: str = Query(..., description="정렬 필드", example="createdAt")
#     order: str = Query(..., description="정렬 순서", example="desc")

class Response_Move_LogsPD(BaseModel):
    items: list[dict] = Field(..., description="로그 목록", example=[{"id": "1", "createdAt": "2021-01-01 12:00:00", "message": "로그 메시지"}])
    pageInfo: PageInfo = Field(..., description="페이지 정보", example={"mode": "offset", "page": 1, "pages": 10, "limit": 10, "total": 100, "sort": "createdAt", "order": "desc"})

class Request_Move_ArchiveLogPD(BaseModel):
    cutOffDate: datetime | date | None = Field(None, example="2026-01-01")
    isDryRun: bool | None = Field(None, example=False)

class Response_Move_ArchiveLogPD(BaseModel):
    cutOffDate: datetime | date = Field(..., description="아카이브 기준 날짜(해당날짜기준부터 이전날짜를 모두 아카이브합니다)",example="2025-11-04")
    isDryRun: bool = Field(..., description="값이 True이면 실제 기능을 수행하진 않고 예상 결과를 반환합니다", example=False)

class Request_Move_ArchiveLogFilterPD(BaseModel):
    cutOffDate: datetime | date = Field(..., description="아카이브 기준 날짜(해당날짜기준부터 이전날짜를 모두 아카이브합니다)",example="2025-11-04")
    filters: str | dict[str, Any] | None = Field(..., description="", example="{'result': 'success'}")
    isDryRun: bool = Field(..., description="값이 True이면 실제 기능을 수행하진 않고 예상 결과를 반환합니다", example=False)

class Request_Move_ExportLogPD(BaseModel):
    startDt: datetime | date | None = Field(None,example="2026-01-01")
    endDt: datetime | date | None = Field(None,example="2026-12-31")
    filters: dict[str, Any] | None = Field(None, example={"result": "success"})
    filename: str | None = Field(None, example="move_logs")
    fields: dict[str, Any] | None = Field(None, example={'id': 0})
    searchText: str | None = Field(None, example="test")
    sort: str = Field(None, example="createdAt")
    order: Literal["asc", "desc"] = Field(None, example="desc")
    method: Literal["file", "email"] = Field(None, example="file")
    email: str | None = Field(None, example="test@example.com")

class Response_Move_ExportLogPD(BaseModel):
    startDt: datetime | date = Field(..., description="내보내기 기준 시작 날짜(해당날짜기준부터 내보냅니다)",example="2025-11-04")
    endDt: datetime | date = Field(..., description="내보내기 기준 종료 날짜(해당날짜기준까지 내보냅니다)",example="2025-11-04")
    filters: str | dict[str, Any] | None = Field(..., description="", example="{'result': 'success'}")
    filename: str = Field(..., description="내보내기 파일명(확장자는 gz로 고정됩니다)", example="move_logs")
    method: str = Field(..., description="내보내기 방식", example="stream")
    fields: str | dict[str, Any] | None = Field(None, description="반환 필드. 입력이 없으면 저장된 모든 필드가 반환되며 입력한 필드의 값에 따라 특정 필드만 반환받거나 특정 필드를 제외하고 반환받고 싶을때 사용하세요. \n 예) id필드만 제외하고 반환 {'id': 0}, id와 status필드만 반환 {'id': 0}", example={'id': 0})
