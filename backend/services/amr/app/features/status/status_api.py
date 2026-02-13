from fastapi import APIRouter

from app.features.status.status_service import AmrStatusService

amr_status_router = APIRouter(
    tags=["AMR 상태"],
    prefix="",
)

amr_status_service = AmrStatusService()

@amr_status_router.get("/{robot_model}/{robot_id}/status", summary="AMR 상태 조회", description="AMR 상태 조회")
async def get_status(robot_model: str, robot_id: str):
    return amr_status_service.get_status(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/moveStatus", summary="AMR 이동 상태 조회", description="AMR 이동 상태 조회")
async def get_move_status(robot_model: str, robot_id: str):
    return amr_status_service.get_move_status(robot_model, robot_id)

@amr_status_router.get("/{robot_model}/{robot_id}/lidar2d", summary="AMR 라이다 2D 조회", description="AMR 라이다 2D 조회")
async def get_lidar2d(robot_model: str, robot_id: str):
    return amr_status_service.get_lidar2d(robot_model, robot_id)
