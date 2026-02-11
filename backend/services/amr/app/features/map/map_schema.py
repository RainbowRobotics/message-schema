from pydantic import BaseModel, Field
from .domain.map import MapFile
from .domain.map import Node

class ResponseMapListPD(BaseModel):
    list: list[MapFile]
    result: str
    message: str | None

class RequestMapDeletePD(BaseModel):
    mapName: str

class ResponseMapDeletePD(BaseModel):
    mapName: str | None
    result: str | None
    message: str | None

class ResponseMapCurrentPD(BaseModel):
    mapName: str | None
    result: str | None
    message: str | None

class RequestGetMapCloudPD(BaseModel):
    mapName: str = Field(..., description="맵 이름", example="Test")
    fileName: str = Field(..., description="파일 이름", example="cloud.csv")

class ResponseGetMapCloudPD(BaseModel):
    mapName: str | None
    fileName: str | None
    data: list[list[float]] | None
    columnCount: int | None
    rowCount: int | None
    result: str | None
    message: str | None

class RequestSetMapCloudPD(BaseModel):
    mapName: str
    fileName: str
    data: list[list[float]]

class ResponseSetMapCloudPD(BaseModel):
    mapName: str | None
    fileName: str | None
    result: str | None
    message: str | None

class RequestGetMapTopologyPD(BaseModel):
    mapName: str
    fileName: str
    pageNo: int
    pageSize: int
    totalPage: int
    nodeType: str
    searchText: str
    sortOption: str
    sortDirection: str
class ResponseGetMapTopologyPD(BaseModel):
    mapName: str | None
    fileName: str | None
    data: list[Node] | None
    pageNo: int | None
    pageSize: int | None
    totalPage: int | None
    nodeType: str | None
    searchText: str | None
    sortOption: str | None
    sortDirection: str | None
    result: str | None
    message: str | None

class RequestSetMapTopologyPD(BaseModel):
    mapName: str
    fileName: str
    data: list[Node] | None

class ResponseSetMapTopologyPD(BaseModel):
    mapName: str | None
    fileName: str | None
    result: str | None
    message: str | None

class RequestMapLoadPD(BaseModel):
    mapName: str

class ResponseMapLoadPD(BaseModel):
    mapName: str | None
    result: str | None
    message: str | None

class ResponseTopoLoadPD(BaseModel):
    mapName: str | None
    result: str | None
    message: str | None

class RequestMapFilePD(BaseModel):
    mapName: str
    fileName: str

class ResponseMapFilePD(BaseModel):
    mapName: str | None
    fileName: str | None
    data: list[bytes] | None
    result: str | None
    message: str | None

class ResponseMappingStartPD(BaseModel):
    result: str | None
    message: str | None
class ResponseMappingStopPD(BaseModel):
    result: str | None
    message: str | None

class RequestMappingSavePD(BaseModel):
    mapName: str

class ResponseMappingSavePD(BaseModel):
    mapName: str | None
    result: str | None
    message: str | None
