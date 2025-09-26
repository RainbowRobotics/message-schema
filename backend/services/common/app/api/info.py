from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from rb_database.mongo_db import MongoDB
from rb_resources.json import read_json_file

info_router = APIRouter(tags=["Info"])


class RobotModelInfo(BaseModel):
    model: str
    name: str
    alias: str
    be_service: list[str]
    useCharge: bool
    urdfFileName: str
    components: list[str]


class RobotInfo(BaseModel):
    serialNumber: str | None = ""
    robotModel: str | None = ""
    robotName: str | None = ""
    components: list[str] | None = []


class RobotInfoResponse(BaseModel):
    info: RobotInfo
    modelInfo: RobotModelInfo | None


@info_router.get("/robot-info", response_model=RobotInfoResponse)
async def get_robot_info(db: MongoDB):
    robot_models = read_json_file("robot_models.json")

    if "robot_info" not in await db.list_collection_names():
        await db.create_collection("robot_info")

    doc = await db["robot_info"].find_one({}, {"_id": 0})

    if not doc:
        doc = dict(RobotInfo())

    robot_model_key = doc.get("robotModel")

    model_info = robot_models.get(robot_model_key)

    res = {"info": doc, "modelInfo": model_info}

    return JSONResponse(res)
