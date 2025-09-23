from enum import StrEnum

from app.modules.whomai.whoami_module_schema import Response_CallWhoamIPD
from pydantic import BaseModel


class ConnectStatus(StrEnum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    UNSTABLE = "UNSTABLE"


class CoreSWWhoamIWithConnectStatus(Response_CallWhoamIPD):
    connected: ConnectStatus


class AllSwConnectStateResponsePD(BaseModel):
    ip: str | None = None
    all_connected: ConnectStatus
    cpu_usage: float
    core_sw_num: int
    core_sw_list: list[CoreSWWhoamIWithConnectStatus]


class PowerControlRequestPD(BaseModel):
    power_option: int
    sync_servo: bool | None = None
    stoptime: float | None = 0.5


class ServoControlRequestPD(BaseModel):
    servo_option: int
