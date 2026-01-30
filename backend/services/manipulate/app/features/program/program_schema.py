from pydantic import BaseModel


class Request_MotionSpeedBarPD(BaseModel):
    alpha: float


class Request_ResumePD(BaseModel):
    pass


class Request_PausePD(BaseModel):
    pass


class Request_MotionHaltPD(BaseModel):
    pass


class Request_ProgramBeforePD(BaseModel):
    option: int


class Request_ProgramAfterPD(BaseModel):
    option: int


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


class Request_Move_SmoothJogJPD(BaseModel):
    target: MoveInputTarget


class Request_Move_SmoothJogLPD(BaseModel):
    target: MoveInputTarget


class Request_Move_SmoothJogStopPD(BaseModel):
    stoptime: float


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

class Request_GetRelativeValuePD(BaseModel):
    relative_value: MoveInputTarget
    reference_value: MoveInputTarget
    move_type: int

class Response_GetRelativeValuePD(BaseModel):
    calculated_result: int
    calculated_value: MoveInputTarget

class Request_RelativeMovePD(BaseModel):
    relative_value: MoveInputTarget
    reference_value: MoveInputTarget
    speed: MoveInputSpeed
    move_type: int

class Request_MoveApproachJPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed

class Request_MoveApproachLPD(BaseModel):
    target: MoveInputTarget
    speed: MoveInputSpeed

class Request_MoveApproachStopPD(BaseModel):
    stoptime: float
