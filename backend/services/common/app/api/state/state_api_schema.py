from pydantic import BaseModel


class PowerControlRequestPD(BaseModel):
    power_option: int
    sync_servo: bool | None = None
    stoptime: float | None = 0.5


class ServoControlRequestPD(BaseModel):
    servo_option: int
