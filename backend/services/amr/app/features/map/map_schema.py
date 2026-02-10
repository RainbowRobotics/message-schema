from pydantic import BaseModel


class MapFileInfo(BaseModel):
    map_name: str
    created_at: str
    update_at: str
    map_type: str
    map_size: float

class MapFile(BaseModel):
    file_name: str
    created_at: str
    update_at: str
    file_type: str
    cloud_info: MapFileInfo
    topo_info: MapFileInfo


class RequestMapListPD(BaseModel):
    id: str

class ResponseMapListPD(BaseModel):
    id: str
    list: list[MapFile]
    result: str
    message: str

class ResponseMapCloudPD(BaseModel):
    mapName: str | None
    fileName: str | None
    data: list[list[float]] | None
    columnCount: int | None
    rowCount: int | None
    result: str | None
    message: str | None
