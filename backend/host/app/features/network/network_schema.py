"""
[Network 모델]
"""

from pydantic import BaseModel, Field
from .domain.network import Network, Wifi



### ======================= Network Get Network ============================
class Response_Network_GetNetworkPD(BaseModel):
    """
    현재 네트워크 조회 응답 (요청은 따로 없음)
    """
    ethernet: Network | None = None
    wifi: Network | None = None
    bluetooth: Network | None = None

### ======================= Network Set IP ============================
class Request_Network_SetPD(BaseModel):
    """
    네트워크 수정 요청
    """
    ssid: str = Field(..., example="mobile_team")
    dhcp: bool = True
    address: str = Field(None, example="10.108.1.20")
    gateway: str = Field(None, example="10.108.1.1")
    netmask: str = Field(None, example="255.255.255.0")
    dns: list[str] = Field(None, example=["8.8.8.8", "8.8.4.4"])

class Response_Network_SetPD(BaseModel):
    ssid: str
    dhcp: bool = False
    address: str | None = None
    gateway: str | None = None
    netmask: str | None = None
    dns: list[str] | None = None
    result: str
    message: str | None = None

### ======================= Network Connect Wifi ============================
class Request_Network_ConnectWifiPD(BaseModel):
    """
    Wifi 연결 요청
    """
    ssid: str = Field(..., example="mobile_team")
    password: str | None = Field(None, example="")

class Response_Network_ConnectWifiPD(BaseModel):
    """
    Wifi 연결 응답
    """
    ssid: str
    address: str | None = None
    gateway: str | None = None
    netmask: str | None = None
    dns: list[str] | None = None
    signal: int | None = None
    result: str
    message: str | None = None


### ======================= Network Get Wifi List ============================
class Request_Network_GetWifiListPD(BaseModel):
    rescan: bool = False

class Response_Network_GetWifiListPD(BaseModel):
    list: list[Wifi]
    result: str
    message: str | None = None
