
from pydantic import (
    BaseModel,
    Field,
)


class Request_Control_OnOffPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    onoff: bool = Field(..., description="LED 켜기/끄기", example=True)
    frequency: int = Field(..., description="주기", example=10)

class Response_Control_OnOffPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    onoff: bool = Field(..., description="LED 켜기/끄기", example=True)
    frequency: int = Field(..., description="주기", example=10)
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_WorkPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    work: str = Field(..., description="작업", example="lidarOn")

class Response_Control_WorkPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    work: str = Field(..., description="작업", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Response_Control_DockPD(BaseModel):
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Response_Control_UndockPD(BaseModel):
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Response_Control_DockStopPD(BaseModel):
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Response_Control_ChargeTriggerPD(BaseModel):
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_SafetyFieldPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyField: int = Field(..., description="안전 필드", example=1)

class Response_Control_SafetyFieldPD(BaseModel):
    safetyField: int = Field(..., description="안전 필드", example=1)
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_SafetyFlagPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyFlag: str = Field(..., description="안전 플래그", example="lidarOn")

class Response_Control_SafetyFlagPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyFlag: str = Field(..., description="안전 플래그", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_LEDPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    led: str = Field(..., description="LED", example="lidarOn")

class Response_Control_LEDPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    led: str = Field(..., description="LED", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_SafetyIOPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyIo: str = Field(..., description="안전 IO", example="lidarOn")

class Response_Control_SafetyIOPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyIo: str = Field(..., description="안전 IO", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_ObsBoxPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    obsBox: str = Field(..., description="관측 박스", example="lidarOn")

class Response_Control_ObsBoxPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    obsBox: str = Field(..., description="관측 박스", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")

class Request_Control_DetectPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    detect: str = Field(..., description="검출", example="lidarOn")

class Response_Control_DetectPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    detect: str = Field(..., description="검출", example="lidarOn")
    result: str = Field(..., description="결과", example="success")
    message: str = Field(..., description="메시지", example="LED 켜기/끄기 성공")
