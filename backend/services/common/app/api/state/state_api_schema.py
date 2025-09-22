from enum import StrEnum

from pydantic import BaseModel


class ConnectStatus(StrEnum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    UNSTABLE = "UNSTABLE"


class CoreSWWhoamI(BaseModel):
    category: str
    name: str
    model: str
    version: str
    alias: str


class CoreSWWhoamIWithConnectStatus(CoreSWWhoamI):
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
