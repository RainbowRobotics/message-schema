from pydantic import BaseModel


class SpeedBarRequestPD(BaseModel):
    speedbar: float


class ResumeRequestPD(BaseModel):
    pass


class PauseRequestPD(BaseModel):
    pass


class SmoothJogJRequestPD(BaseModel):
    targetspeed: list[float]
    frame: int
    unit: int


class SmoothJogLRequestPD(BaseModel):
    targetspeed: list[float]
    frame: int
    unit: int


class SmoothJogStopRequestPD(BaseModel):
    stoptime: float


class MoveInputTarget(BaseModel):
    tar_values: list[float]
    tar_frame: int
    tar_unit: int


class MoveInputSpeed(BaseModel):
    spd_mode: int
    spd_vel_para: float
    spd_acc_para: float


class MoveInputType(BaseModel):
    pnt_type: int
    pnt_para: float


class Request_MoveJPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed


class Request_MoveLPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed


class Request_MoveTickJogJPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed


class Request_MoveTickJogLPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed


class Request_MoveJBAddPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed
    type: MoveInputType


class Request_MoveLBAddPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed
    type: MoveInputType


class Request_MoveLBRunPD(BaseModel):
    orientation: int

class Request_MoveXBAddPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed
    type: MoveInputType
    method: int


class Request_MoveXBRunPD(BaseModel):
    running_mode: int