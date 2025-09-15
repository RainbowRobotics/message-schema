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
