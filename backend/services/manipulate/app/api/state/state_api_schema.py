from pydantic import BaseModel


class PowerControlRequestPD(BaseModel):
    power_option: int
    sync_servo: bool


class ServoControlRequestPD(BaseModel):
    servo_option: int


class ReferenceControlRequestPD(BaseModel):
    reference_option: int
