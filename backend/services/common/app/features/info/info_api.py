from fastapi import APIRouter
from fastapi.responses import JSONResponse
from rb_database.mongo_db import MongoDB

from .info_module import InfoService
from .info_schema import (
    Request_Create_Robot_InfoPD,
    Response_RobotURDFLinkMapPD,
    RobotInfo,
    RobotInfoResponse,
)

info_service = InfoService()
info_router = APIRouter(tags=["Info"])


@info_router.get("/robot-info", response_model=RobotInfoResponse)
async def get_robot_info(db: MongoDB):
    return await info_service.get_robot_info(db=db)


@info_router.get("/{robot_model}/robot-urdf-link-map", response_model=Response_RobotURDFLinkMapPD)
async def get_robot_urdf_link_map(robot_model: str):
    return await info_service.get_robot_urdf_link_map(robot_model=robot_model)


@info_router.get("/robot-category-list", response_model=list[str])
async def get_robot_category_list(db: MongoDB):
    return await info_service.get_robot_category_list(db=db)


@info_router.get("/robot-component-list", response_model=list[str])
async def get_robot_component_list(db: MongoDB, be_service: str | None = None):
    return await info_service.get_robot_component_list(db=db, be_service=be_service)


@info_router.post("/robot-info", response_model=list[str])
async def insert_robot_info(db: MongoDB, request: Request_Create_Robot_InfoPD):
    res = await info_service.insert_robot_info(db=db, request=request)
    return JSONResponse(content=res)


@info_router.put("/robot-info", response_model=list[str])
async def update_robot_info(db: MongoDB, robot_info: RobotInfo):
    return await info_service.update_robot_info(db=db, robot_info=robot_info)