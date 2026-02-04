
from pydantic import (
    BaseModel,
    Field,
)


class ControlLEDResponse(BaseModel):
    id: str
    onoff: bool
    color: str
    result: str | None = None
    message: str | None = None

class Request_Control_FrequencyPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    target: str = Field(None, example="lidar")
    onoff: bool = Field(None, example=True)
    frequency: int = Field(None, example=10)

class Request_Control_OnOffPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    command: str = Field(..., description="명령", example="lidarOnOff")
    onoff: bool = Field(..., description="LED 켜기/끄기", example=True)
    frequency: int = Field(..., description="주기", example=10)

class Response_Control_OnOffPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    onoff: bool = Field(..., description="LED 켜기/끄기", example=True)
    frequency: int = Field(..., description="주기", example=10)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_WorkPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    work: str = Field(..., description="작업", example="lidarOn")

class Request_Control_DockPD(BaseModel):
    command: str = Field(..., description="도킹 명령", example="dock")

class Response_Control_WorkPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    work: str = Field(..., description="작업", example="lidarOn")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Response_Control_DockPD(BaseModel):
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Response_Control_UndockPD(BaseModel):
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Response_Control_DockStopPD(BaseModel):
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Response_Control_ChargeTriggerPD(BaseModel):
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_SafetyFieldPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    safetyField: int = Field(..., description="안전 필드", example=1)

class Response_Control_SafetyFieldPD(BaseModel):
    safetyField: int = Field(..., description="안전 필드", example=1)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_SafetyFlagPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyFlag: str = Field(..., description="안전 플래그", example="lidarOn")

class Response_Control_SafetyFlagPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    safetyFlag: str = Field(..., description="안전 플래그", example="lidarOn")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_LEDPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    command: str = Field(..., description="명령", example="lidarOnOff")
    led: str = Field(..., description="LED", example="lidarOn")

class Request_Control_MotorPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    onoff: bool = Field(..., description="모터", example=True)
class Response_Control_LEDPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    led: str = Field(..., description="LED", example="lidarOn")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_SafetyIOPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    command: str = Field(..., description="명령", example="lidarOnOff")
    mcu0_dio: list[bool] = Field(..., description="MCU0 DIO", example=[True, False, True])
    mcu1_dio: list[bool] = Field(..., description="MCU1 DIO", example=[True, False, True])

class Response_Control_SafetyIOPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    mcu0_dio: list[bool] = Field(..., description="MCU0 DIO", example=[True, False, True])
    mcu1_dio: list[bool] = Field(..., description="MCU1 DIO", example=[True, False, True])
    mcu0_din: list[bool] = Field(..., description="MCU0 DIN", example=[True, False, True])
    mcu1_din: list[bool] = Field(..., description="MCU1 DIN", example=[True, False, True])
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_ObsBoxPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    command: str = Field(..., description="명령", example="lidarOnOff")
    obsBox: str = Field(..., description="관측 박스", example="lidarOn")

class Response_Control_ObsBoxPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    obsBox: str = Field(..., description="관측 박스", example="lidarOn")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class Request_Control_DetectPD(BaseModel):
    robot_model: str = Field(..., description="로봇 모델", example="test")
    command: str = Field(..., description="명령", example="lidarOnOff")
    detect: str = Field(..., description="검출", example="lidarOn")

class Response_Control_DetectPD(BaseModel):
    command: str = Field(..., description="명령", example="lidarOnOff")
    detect: str = Field(..., description="검출", example="lidarOn")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")
