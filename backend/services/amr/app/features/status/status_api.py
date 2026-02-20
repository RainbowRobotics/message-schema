from fastapi import APIRouter
from rb_schemas.fbs_models.amr.v1.slamnav_status_models import StatusPD
from rb_utils.service_exception import ServiceException

from app.features.status.status_service import AmrStatusService

amr_status_router = APIRouter(
    tags=["AMR 상태"],
    prefix="",
)

amr_status_service = AmrStatusService()

@amr_status_router.get("/{robot_model}/{robot_id}/status", summary="AMR 상태 조회", description="AMR 상태 조회")
async def get_status(robot_model: str, robot_id: str) -> StatusPD:
    return await amr_status_service.get_status(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/moveStatus", summary="AMR 이동 상태 조회", description="AMR 이동 상태 조회")
async def get_move_status(robot_model: str, robot_id: str):
    return await amr_status_service.get_move_status(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/lidar2d", summary="AMR 라이다 2D 조회", description="AMR 라이다 2D 조회")
async def get_lidar2d(robot_model: str, robot_id: str):
    return await amr_status_service.get_lidar2d(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/lidar3d", summary="AMR 라이다 3D 조회", description="AMR 라이다 3D 조회")
async def get_lidar3d(robot_model: str, robot_id: str):
    raise ServiceException("AMR 라이다 3D 조회 기능은 현재 지원하지 않습니다", status_code=500)
    # return amr_status_service.get_lidar3d(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/globalPath", summary="AMR 전역 경로 조회", description="AMR 전역 경로 조회")
async def get_global_path(robot_model: str, robot_id: str):
    return await amr_status_service.get_global_path(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/localPath", summary="AMR 지역 경로 조회", description="AMR 지역 경로 조회")
async def get_local_path(robot_model: str, robot_id: str):
    return await amr_status_service.get_local_path(robot_model, robot_id)
