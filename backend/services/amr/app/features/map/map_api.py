
from fastapi import APIRouter, BackgroundTasks

from .map_service import AmrMapService

amr_map_router = APIRouter(
    tags=["AMR 맵"],
    prefix="",
)

amr_map_service = AmrMapService()

@amr_map_router.get("/{robot_model}/map/list", summary="맵 목록 조회", description="맵 목록 조회")
async def get_map_list(robot_model: str):
    return await amr_map_service.get_map_list(robot_model)

@amr_map_router.get("/{robot_model}/map/cloud", summary="맵 클라우드 조회", description="맵 클라우드 조회")
async def get_map_cloud(robot_model: str, map_name: str, file_name: str):
    return await amr_map_service.get_map_cloud(robot_model, map_name, file_name)
