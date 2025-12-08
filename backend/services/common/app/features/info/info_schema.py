# mypy: disable-error-code=misc
from typing import Annotated, Literal

from pydantic import BaseModel, Field
from rb_schemas.utility import Omit


class RobotModelInfo(BaseModel):
    model: str
    alias: str
    be_service: str
    useCharge: bool | None = False
    components: list[str]


class RobotInfo(BaseModel):
    serialNumber: str | None = Field(default=None)
    robotModel: str | None = Field(default="")
    robotName: str | None = Field(default=None)
    programId: str | None = Field(default=None)
    components: list[str] | None = Field(default=[])


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


class Request_Upsert_Robot_InfoPD(Omit(RobotInfo, "components")):
    robotModel: str | None = Field(default=None)
