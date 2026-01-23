"""
[Network 모델]
"""


from dataclasses import dataclass

from pydantic import BaseModel, Field


@dataclass(frozen=True)
class Wifi:
    in_use: bool
    ssid: str
    signal: int
    security: str
    channel: int
    rate: str

@dataclass(frozen=True)
class Network:
    device: str
    dhcp: bool
    dns: list[str]             # IPv4 dns list
    ssid: str | None        # wifi만
    address: str | None          # IPv4
    netmask: str | None     # IPv4 netmask
    gateway: str | None     # IPv4 gateway
    signal: int | None      # wifi만


### ======================= Network Get Network ============================
class Response_Network_GetNetwork(BaseModel):
    ethernet: Network | None = None
    wifi: Network | None = None
    bluetooth: Network | None = None

### ======================= Network Set IP ============================
class Request_Network_SetPD(BaseModel):
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
class Request_Network_ConnectWifi(BaseModel):
    ssid: str = Field(..., example="mobile_team")
    password: str | None = Field(None, example="")

class Response_Network_ConnectWifi(BaseModel):
    ssid: str
    password: str | None = None
    address: str | None = None
    gateway: str | None = None
    netmask: str | None = None
    dns: list[str] | None = None
    signal: int | None = None
    result: str
    message: str | None = None


### ======================= Network Get Wifi List ============================
class Request_Network_GetWifiList(BaseModel):
    rescan: bool = False

class Response_Network_GetWifiList(BaseModel):
    list: list[Wifi]
    result: str
    message: str | None = None
