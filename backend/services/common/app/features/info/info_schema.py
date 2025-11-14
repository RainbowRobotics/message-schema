from typing import Annotated, Literal

from pydantic import BaseModel, Field


class RobotModelInfo(BaseModel):
    model: str
    alias: str
    be_service: str
    useCharge: bool | None = False
    components: list[str]


class RobotInfo(BaseModel):
    serialNumber: str | None = ""
    robotModel: str | None = ""
    robotName: str | None = ""
    programId: str | None = None
    components: list[str] | None = []


class URDFLinkMapArray(BaseModel):
    type: Literal["array"]
    beService: str
    mapper: list[str]


class URDFLinkMapObject(BaseModel):
    type: Literal["object"]
    beService: str
    mapper: dict[str, str]


ST_URDF_Link_Map_ComponentPD = Annotated[
    URDFLinkMapArray | URDFLinkMapObject,
    Field(discriminator="type"),
]


class RobotInfoResponse(BaseModel):
    info: RobotInfo
    modelInfo: RobotModelInfo | None


class Response_RobotURDFLinkMapPD(BaseModel):
    urdfFileName: str
    components: dict[str, ST_URDF_Link_Map_ComponentPD]
