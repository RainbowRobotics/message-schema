from pydantic import BaseModel, Field
from rb_database.schema import PyObjectId


class RealTimeLogItem(BaseModel):
    swName: str
    level: int
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


class Response_LogListPD(BaseModel):
    items: list[LogItem]
    # nextCursor: Cursor | None
