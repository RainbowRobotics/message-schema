from pydantic import BaseModel, Field
from rb_database.schema import PyObjectId


class RealTimeLogItem(BaseModel):
    swName: str | None = None
    level: int | str
    timestamp: str
    contents: str
    createdAt: str


class LogItem(RealTimeLogItem):
    id: PyObjectId = Field(alias="_id")  # Mongo 키를 id로 노출


class Response_LogCntPD(BaseModel):
    count: int


class Cursor(BaseModel):
    createdAt: str
    id: PyObjectId


class Request_LogListParamsPD(BaseModel):
    limit: int | None = None
    pageNum: int | None = None
    level: list[int | str] | int | str | None = None
    swName: str | None = None
    searchText: str | None = None
    fromDate: str | None = None
    toDate: str | None = None


class Response_LogListPD(BaseModel):
    items: list[LogItem]
    hasNext: bool
    totalCount: int
    # nextCursor: Cursor | None


class Request_ExportStateLogsParamsPD(BaseModel):
    fromDate: str
    toDate: str
    swName: str | None = None
    searchText: str | None = None
    level: list[int | str] | int | str | None = None
    filename: str | None = None
