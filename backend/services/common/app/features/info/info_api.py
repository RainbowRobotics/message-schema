from fastapi import APIRouter
from rb_database.mongo_db import MongoDB

from .info_module import InfoService
from .info_schema import RobotInfoResponse

info_service = InfoService()
info_router = APIRouter(tags=["Info"])


@info_router.get("/robot-info", response_model=RobotInfoResponse)
async def get_robot_info(db: MongoDB):
    return await info_service.get_robot_info(db=db)
