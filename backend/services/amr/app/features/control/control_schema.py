
from pydantic import (
    BaseModel,
    Field,
)
from rb_sdk.amr_sdk.amr_control import SafetyFlag

class RequestGetSafetyFieldPD(BaseModel):
    """
    [세이프티 필드 조회 요청]
    """

class ResponseGetSafetyFieldPD(BaseModel):
    """
    [충전 트리거 명령 응답]
    * safetyField : 세이프티 영역 번호
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    safetyField: int = Field(..., example=1)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestSetSafetyFieldPD(BaseModel):
    """
    [세이프티 필드 설정 요청]
    * safetyField : 세이프티 영역 번호
    """
    safetyField: int = Field(..., example=1)

class ResponseSetSafetyFieldPD(BaseModel):
    """
    [세이프티 필드 설정 응답]
    * safetyField : 세이프티 영역 번호
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    safetyField: int = Field(..., example=1)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestGetSafetyFlagPD(BaseModel):
    """
    [세이프티 필드 조회 요청]
    """
class ResponseGetSafetyFlagPD(BaseModel):
    """
    [세이프티 필드 조회 응답]
    * resetFlag : 세이프티 플래그 목록
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    resetFlag: list[SafetyFlag] = Field(..., example=[{"name":"obstacle","value":False}])
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestSetSafetyFlagPD(BaseModel):
    """
    [세이프티 필드 설정 요청]
    * safetyFlag : 세이프티 플래그 목록
    """
    safetyFlag: list[SafetyFlag] = Field(..., example=[{"name":"obstacle","value":False}])

class ResponseSetSafetyFlagPD(BaseModel):
    """
    [세이프티 필드 설정 응답]
    * safetyFlag : 세이프티 플래그 목록
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    safetyFlag: list[SafetyFlag] = Field(..., example=[{"name":"obstacle","value":False}])
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestGetSafetyIoPD(BaseModel):
    """
    [세이프티 IO 조회 요청]
    """
class ResponseGetSafetyIoPD(BaseModel):
    """
    [세이프티 IO 조회 응답]
    # * mcu0Dio : MCU0 Digital Output (8bit)
    # * mcu1Dio : MCU1 Digital Output (8bit)
    * mcu0Din : MCU0 Digital Input (8bit)
    * mcu1Din : MCU1 Digital Input (8bit)
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    # mcu0Dio: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    # mcu1Dio: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    mcu0Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    mcu1Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestSetSafetyIoPD(BaseModel):
    """
    [세이프티 IO 설정 요청]
    * mcu0Din : MCU0 Digital Input (8bit)
    * mcu1Din : MCU1 Digital Input (8bit)
    """
    mcu0Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    mcu1Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])

class ResponseSetSafetyIoPD(BaseModel):
    """
    [세이프티 IO 설정 응답]
    * mcu0Din : MCU0 Digital Input (8bit)
    * mcu1Din : MCU1 Digital Input (8bit)
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    mcu0Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    mcu1Din: list[bool] = Field(..., example=[False, True, False, True, False, True, False, True])
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlDockPD(BaseModel):
    """
    [도킹 명령 요청]
    * command : 도킹 명령 ( "dock", "undock", "dockstop" )
    """
    command: str = Field(..., description="도킹 명령", example="dock")
class ResponseControlDockPD(BaseModel):
    """
    [도킹 명령 응답]
    * command : 도킹 명령 ( "dock", "undock", "dockstop" )
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    command: str = Field(..., description="도킹 명령", example="dock")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlChargeTriggerPD(BaseModel):
    """
    [충전 트리거 명령 요청]
    * control : 충전 트리거 명령 ( True / False )
    """
    control: bool = Field(..., example=True)
class ResponseControlChargeTriggerPD(BaseModel):
    """
    [충전 트리거 명령 응답]
    * control : 충전 트리거 명령 ( True / False )
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    control: bool = Field(..., example=True)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlGetObsBoxPD(BaseModel):
    """
    [옵스 박스 조회 요청]
    """

class ResponseControlGetObsBoxPD(BaseModel):
    """
    [옵스 박스 조회 응답]
    * min_x : 옵스 박스 최소 x 범위
    * min_y : 옵스 박스 최소 y 범위
    * min_z : 옵스 박스 최소 z 범위
    * max_x : 옵스 박스 최대 x 범위
    * max_y : 옵스 박스 최대 y 범위
    * max_z : 옵스 박스 최대 z 범위
    * map_range : 옵스 박스 맵 범위
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    min_x: float = Field(..., example=1.0)
    min_y: float = Field(..., example=1.0)
    min_z: float = Field(..., example=1.0)
    max_x: float = Field(..., example=1.0)
    max_y: float = Field(..., example=1.0)
    max_z: float = Field(..., example=1.0)
    map_range: float = Field(..., example=1.0)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlSetObsBoxPD(BaseModel):
    """
    [옵스 박스 설정 요청]
    * min_x : 옵스 박스 최소 x 범위
    * min_y : 옵스 박스 최소 y 범위
    * min_z : 옵스 박스 최소 z 범위
    * max_x : 옵스 박스 최대 x 범위
    * max_y : 옵스 박스 최대 y 범위
    * max_z : 옵스 박스 최대 z 범위
    * map_range : 옵스 박스 맵 범위
    """
    min_x: float = Field(..., example=1.0)
    min_y: float = Field(..., example=1.0)
    min_z: float = Field(..., example=1.0)
    max_x: float = Field(..., example=1.0)
    max_y: float = Field(..., example=1.0)
    max_z: float = Field(..., example=1.0)
    map_range: float = Field(..., example=1.0)

class ResponseControlSetObsBoxPD(BaseModel):
    """
    [옵스 박스 설정 응답]
    * min_x : 옵스 박스 최소 x 범위
    * min_y : 옵스 박스 최소 y 범위
    * min_z : 옵스 박스 최소 z 범위
    * max_x : 옵스 박스 최대 x 범위
    * max_y : 옵스 박스 최대 y 범위
    * max_z : 옵스 박스 최대 z 범위
    * map_range : 옵스 박스 맵 범위
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    min_x: float = Field(..., example=1.0)
    min_y: float = Field(..., example=1.0)
    min_z: float = Field(..., example=1.0)
    max_x: float = Field(..., example=1.0)
    max_y: float = Field(..., example=1.0)
    max_z: float = Field(..., example=1.0)
    map_range: float = Field(..., example=1.0)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlLedModePD(BaseModel):
    """
    [LED 모드 설정 요청]
    * control : LED on / off
    * color : LED color
    """
    control: bool = Field(..., example=True)
    color: str = Field(..., example="red")

class ResponseControlLedModePD(BaseModel):
    """
    [LED 모드 설정 응답]
    * control : LED on / off
    * color : LED color
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    control: bool = Field(..., example=True)
    color: str = Field(..., example="red")
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlMotorModePD(BaseModel):
    """
    [모터 모드 설정 요청]
    * control : 모터 on / off
    """
    control: bool = Field(..., example=True)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class ResponseControlMotorModePD(BaseModel):
    """
    [모터 모드 설정 응답]
    * control : 모터 on / off
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    control: bool = Field(..., example=True)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlJogModePD(BaseModel):
    """
    [조그 모드 설정 요청]
    * control : 조그 on / off
    """
    control: bool = Field(..., example=True)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class ResponseControlJogModePD(BaseModel):
    """
    [조그 모드 설정 응답]
    * control : 조그 on / off
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    control: bool = Field(..., example=True)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlSensorModePD(BaseModel):
    """
    [센서 모드 설정 요청]
    * command : 센서 종류 ( "camera", "lidar2d", "lidar3d" )
    * control : 센서 통신 on / off
    * frequency : 센서 주파수 (Hz)
    """
    command: str = Field(..., example="camera")
    control: bool = Field(..., example=True)
    frequency: int = Field(..., example=10)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class ResponseControlSensorModePD(BaseModel):
    """
    [센서 모드 설정 응답]
    * command : 센서 종류 ( "camera", "lidar2d", "lidar3d" )
    * control : 센서 통신 on / off
    * frequency : 센서 주파수 (Hz)
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    command: str = Field(..., example="camera")
    control: bool = Field(..., example=True)
    frequency: int = Field(..., example=10)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlPathModePD(BaseModel):
    """
    [경로 모드 설정 요청]
    * control : 경로 통신 on / off
    * frequency : 경로 주파수 (Hz)
    """
    control: bool = Field(..., example=True)
    frequency: int = Field(..., example=10)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class ResponseControlPathModePD(BaseModel):
    """
    [경로 모드 설정 응답]
    * control : 경로 통신 on / off
    * frequency : 경로 주파수 (Hz)
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    control: bool = Field(..., example=True)
    frequency: int = Field(..., example=10)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")

class RequestControlDetectMarkerPD(BaseModel):
    """
    [장애물 감지 명령 요청]
    * command : 감지 명령 ( "charuco", "aruco" )
    * camera_number : 카메라 번호
    * camera_serial : 카메라 시리얼
    * marker_size : 마커 크기
    """
    command: str = Field(..., example="aruco")
    camera_number: int = Field(..., example=0)
    camera_serial: str = Field(..., example="1234567890")
    marker_size: float = Field(..., example=0.1)

class ResponseControlDetectMarkerPD(BaseModel):
    """
    [장애물 감지 명령 응답]
    * command : 감지 명령 ( "charuco", "aruco" )
    * camera_number : 카메라 번호
    * camera_serial : 카메라 시리얼
    * marker_size : 마커 크기
    * result : 요청한 명령에 대한 결과입니다.
    * message : result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다.
    """
    command: str = Field(..., example="aruco")
    camera_number: int = Field(..., example=0)
    camera_serial: str = Field(..., example="1234567890")
    marker_size: float = Field(..., example=0.1)
    result: str = Field(..., example="accept")
    message: str | None = Field(None, example="")
