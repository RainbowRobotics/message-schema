from enum import StrEnum

from app.modules.whomai.whoami_module_schema import Response_CallWhoamIPD
from pydantic import BaseModel


class Request_System_StatePD(BaseModel):
    namespaces: list[str]


class ConnectStatus(StrEnum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    POWER_CHECKED = "POWER CHECKED"
    COMM_CHECKED = "COMM CHECKED"
    PARAM_CHECKED = "PARAM CHECKED"
    JOINT_CHECKED = "JOINT CHECKED"
    SYSTEM_CHECKED = "SYSTEM CHECKED"
    UNSTABLE = "UNSTABLE"


class Core_SW_Simple_Info(BaseModel):
    sw_name: str
    model: str
    alias: str
    be_service: str
    connected: ConnectStatus


class CoreSWWhoamIWithConnectStatus(Response_CallWhoamIPD):
    connected: ConnectStatus


class AllSwConnectStateResponsePD(BaseModel):
    ip: str | None = None
    all_connected: ConnectStatus
    cpu_usage: float
    core_sw_list: list[Core_SW_Simple_Info]


class PowerControlRequestPD(BaseModel):
    power_option: int
    sync_servo: bool | None = None
    stoptime: float | None = 0.5


class ServoControlRequestPD(BaseModel):
    servo_option: int
