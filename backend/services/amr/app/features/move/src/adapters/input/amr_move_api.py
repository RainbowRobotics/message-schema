"""
[AMR 이동 API 어댑터]
"""
from app.features.move.schema.move_api import (
    Request_Move_GoalPD,
    Request_Move_JogPD,
    Request_Move_TargetPD,
    RequestAmrMoveArchiveLogPD,
    RequestAmrMoveExportLogPD,
    RequestAmrMoveLogsPD,
    Response_Move_GoalPD,
    Response_Move_LogsPD,
    Response_Move_PausePD,
    Response_Move_ResumePD,
    Response_Move_StopPD,
    Response_Move_TargetPD,
)
from app.features.move.src.application.amr_move_service import AmrMoveService
from fastapi import APIRouter

amr_move_router = APIRouter(
    tags=["AMR 이동"],
    prefix="/slamnav/move",
)

amr_move_service = AmrMoveService()

@amr_move_router.post(
    "/goal",
    summary="목표 지점으로 이동",
    description="""
    AMR을 지정된 목표 지점(Goal)으로 이동시킵니다.
    
    - goalId: 이동할 목표 지점의 ID
    - method: 이동 방식 (예: pp)
    - preset: 사전 설정된 이동 프로파일 번호
    """,
    response_description="이동 명령 처리 결과 반환"
)
async def slamnav_move_goal(request: Request_Move_GoalPD) -> Response_Move_GoalPD:
    """
    - request: Request_Move_GoalPD
    - amr_move_service.move_goal: AmrMoveService.move_goal 메서드 호출
    - 이동 명령 처리 결과 반환
    """
    return await amr_move_service.move_goal(request)

@amr_move_router.post(
    "/target",
    summary="타겟 좌표로 이동",
    description="""
    AMR을 지정된 좌표로 이동시킵니다. (현재 작동하지 않습니다)
    
    - goalPose: 이동할 목표 좌표 [x, y, z, rz] (x, y, z: 위치 [m], rz: 회전 [deg])
    - method: 이동 방식 (예: pp)
    - preset: 사전 설정된 이동 프로파일
    
    """,
    response_description="이동 명령 처리 결과 반환"
)
async def slamnav_move_target(request: Request_Move_TargetPD) -> Response_Move_TargetPD:
    """
    - request: Request_Move_TargetPD
    - amr_move_service.move_target: AmrMoveService.move_target 메서드 호출
    - 이동 명령 처리 결과 반환
    """
    return await amr_move_service.move_target(request)

@amr_move_router.post(
    "/jog",
    summary="조이스틱 이동",
    description="""
    AMR을 주어진 속도값으로 조이스틱 이동시킵니다. 반환값은 없습니다
    
    - vx: 직진 속도 (m/s)
    - vy: 직진 속도 (m/s)
    - wz: 회전 속도 (deg/s)
    """,
)
async def slamnav_move_jog(request: Request_Move_JogPD) -> None:
    """
    - request: Request_Move_JogPD
    - amr_move_service.move_jog: amr_move_service.move_jog 메서드 호출
    """
    return await amr_move_service.move_jog(request)

@amr_move_router.post(
    "/stop",
    summary="이동 중지",
    description="AMR을 이동 중지시킵니다.",
    response_description="이동 중지 명령 처리 결과 반환"
)
async def slamnav_move_stop() -> Response_Move_StopPD:
    """
    - request: Request_Move_StopPD
    - amr_move_service.move_stop: amr_move_service.move_stop 메서드 호출
    - 이동 명령 처리 결과 반환
    """
    return await amr_move_service.move_stop()


@amr_move_router.post(
    "/pause",
    summary="이동 일시정지",
    description="AMR을 이동 일시정지시킵니다.",
    response_description="이동 일시정지 명령 처리 결과 반환"
)
async def slamnav_move_pause() -> Response_Move_PausePD:
    """
    - request: Request_Move_PausePD
    - amr_move_service.move_pause: amr_move_service.move_pause 메서드 호출
    - 이동 명령 처리 결과 반환
    """
    return await amr_move_service.move_pause()

@amr_move_router.post(
    "/resume",
    summary="이동 재개",
    description="AMR을 이동 재개시킵니다.",
    response_description="이동 재개 명령 처리 결과 반환"
)
async def slamnav_move_resume() -> Response_Move_ResumePD:
    """
    - request: Request_Move_ResumePD
    - amr_move_service.move_resume: amr_move_service.move_resume 메서드 호출
    - 이동 명령 처리 결과 반환
    """
    return await amr_move_service.move_resume()


@amr_move_router.post(
    "/logs",
    summary="이동 로그 조회",
    description="이동 로그를 조회합니다.",
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
    description="이동 로그를 내보냅니다. 입력된 기간동안의 로그를 내보냅니다",
    response_description="이동 로그 내보내기 결과 반환"
)
async def export_move_logs(request: RequestAmrMoveExportLogPD):
    """
    - request: RequestAmrMoveExportLogPD
    - amr_move_service.export_logs: amr_move_service.export_logs 메서드 호출
    - 이동 로그 내보내기 결과 반환
    """
    return await amr_move_service.export_logs(request)

