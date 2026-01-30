"""
[Network 모델]
"""


from dataclasses import dataclass

from pydantic import BaseModel, Field

### ======================= Sound Play ============================
class Request_Sound_PlayPD(BaseModel):
    fileName: str | None = Field(None, example="test.mp3")
    filePath: str | None = Field(None, example="/home/rainbow/web-rainbow-server/public/sound/test.mp3")
    volume: int = Field(..., example=50)
    repeatCount: int | None = Field(None, example=1)

class Response_Sound_PlayPD(BaseModel):
    fileName: str
    filePath: str
    volume: int
    repeatCount: int
    result: str
    message: str | None = None

### ======================= Sound Stop ============================
class Request_Sound_StopPD(BaseModel):
    pass

class Response_Sound_StopPD(BaseModel):
    result: str
    message: str | None = None

### ======================= Sound Get Status ============================
class Request_Sound_GetStatusPD(BaseModel):
    pass

class Response_Sound_GetStatusPD(BaseModel):
    fileName: str
    status: str
    volume: int
    repeatCount: int
