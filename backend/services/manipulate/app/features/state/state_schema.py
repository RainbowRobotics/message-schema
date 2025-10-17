from pydantic import BaseModel
from rb_schemas.base import (
    NAinPfPD,
    NAOutfPD,
    NCarrefPD,
    NDInuPD,
    NDOutuPD,
    NJointfPD,
    Response_ReturnValuePD,
)


class PowerControlRequestPD(BaseModel):
    power_option: int
    stoptime: float | None = 0.5
    sync_servo: bool | None = True


class ServoControlRequestPD(BaseModel):
    servo_option: int


class ReferenceControlRequestPD(BaseModel):
    reference_option: int


class StateRequestPD(BaseModel):
    jointQRef: NJointfPD
    jointQEnc: NJointfPD
    jointTEsti: NJointfPD
    jointTMeas: NJointfPD
    jointTemper: NJointfPD

    carteXRef: NCarrefPD
    carteXEnc: NCarrefPD

    userfSelectionNo: int
    userfXRef: NCarrefPD

    toolSelectionNo: int
    toolName: str
    toolTcpX: float
    toolTcpY: float
    toolTcpZ: float
    toolTcpRx: float
    toolTcpRy: float
    toolTcpRz: float
    toolComM: float
    toolComX: float
    toolComY: float
    toolComZ: float

    cboxDigitalInput: NDInuPD
    cboxDigitalOutput: NDOutuPD
    cboxAnalogInput: NAinPfPD
    cboxAnalogOutput: NAOutfPD

    exDigitalInput: NDInuPD
    exDigitalOutput: NDOutuPD
    exAnalogInput: NAinPfPD
    exAnalogOutput: NAOutfPD

    toolDigitalInput: NDInuPD
    toolDigitalOutput: NDOutuPD
    toolAnalogInput: NAinPfPD
    toolAnalogOutput: NAOutfPD

    motionMode: int
    motionSpeedBar: float
    motionIsPause: int

    statusLan2can: int
    statusSwitchEmg: int
    statusPowerOut: int
    statusServoNum: int
    statusIsRefon: int
    statusOutColl: int
    statusSelfColl: int
    statusDtMode: int


class PowerControlResponsePD(Response_ReturnValuePD):
    target: str | None = None
