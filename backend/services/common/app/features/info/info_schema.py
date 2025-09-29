from pydantic import BaseModel


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
