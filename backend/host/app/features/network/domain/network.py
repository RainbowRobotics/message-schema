"""
[Network 도메인]
"""
from dataclasses import (
    dataclass,
    field
)
from datetime import (
    UTC,
    datetime,
)
from enum import Enum
from typing import Any, List, Optional
import uuid
from rb_utils.service_exception import ServiceException

from app.features.network.schema.network_dto import Network, Request_Network_ConnectWifi, Request_Network_SetPD


class NetworkCommandEnum(str, Enum):
    """
    [Network 명령]
    """
    GET_NETWORK = "getNetwork"
    SET_NETWORK = "setNetwork"
    CONNECT_WIFI = "connectWifi"



@dataclass
class NetworkModel:
    """
    [Network 모델]
    """
    command: str | None = None

    # Network 정보
    device: str | None = None
    dhcp: bool | None = None
    address: str | None = None
    gateway: str | None = None
    netmask: str | None = None
    dns: list[str] | None = None
    signal: Optional[int] | None = None

    # WiFi 정보
    ssid: str | None = None
    password: str | None = None

    # 결과 정보
    result: str | None = None
    message: str | None = None

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    created_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    update_at: datetime = field(default_factory=lambda: datetime.now(UTC))

    def set_network(self, req: Request_Network_SetPD):
        self.command = NetworkCommandEnum.SET_NETWORK
        self.ssid = req.ssid
        self.dhcp = req.dhcp
        self.address = req.address
        self.gateway = req.gateway
        self.netmask = req.netmask
        self.dns = req.dns
        self.update_at = datetime.now(UTC)

    def set_connect_wifi(self, req: Request_Network_ConnectWifi):
        self.command = NetworkCommandEnum.CONNECT_WIFI
        self.ssid = req.ssid
        self.password = req.password
        self.update_at = datetime.now(UTC)

    def network_info(self, network: Network) -> None:
        self.device = network.device
        self.dns = network.dns
        self.ssid = network.ssid
        self.address = network.address
        self.netmask = network.netmask
        self.gateway = network.gateway
        self.signal = network.signal
        self.update_at = datetime.now(UTC)

    def check_variables(self) -> None:
        """
        [요청 변수 검사]
        """
        if self.command is NetworkCommandEnum.GET_NETWORK:
            pass
        elif self.command is NetworkCommandEnum.SET_NETWORK:
            if self.ssid is None or self.ssid == "":
                raise ServiceException("device 값이 없습니다.", 403)
            if self.dhcp is None:
                raise ServiceException("dhcp 값이 없습니다.", 403)
            if self.address is None:
                raise ServiceException("address 값이 없습니다.", 403)
            if self.gateway is None:
                raise ServiceException("gateway 값이 없습니다.", 403)
            if self.netmask is None:
                raise ServiceException("netmask 값이 없습니다.", 403)
            if self.dns is None:
                raise ServiceException("dns 값이 없습니다.", 403)
        elif self.command is NetworkCommandEnum.CONNECT_WIFI:
            if self.ssid is None or self.ssid == "":
                raise ServiceException("ssid 값이 없습니다.", 403)
        else:
            raise ServiceException("알 수 없는 command 값입니다.", 403)

    def to_dict(self) -> dict[str, Any]:
        """
        - return: dict[str, Any]
        """
        d = {}
        if self.command == NetworkCommandEnum.SET_NETWORK:
            d["ssid"] = self.ssid
            d["dhcp"] = self.dhcp
            d["address"] = self.address
            d["gateway"] = self.gateway
            d["netmask"] = self.netmask
            d["dns"] = self.dns
            d["signal"] = self.signal
        elif self.command == NetworkCommandEnum.CONNECT_WIFI:
            d["ssid"] = self.ssid
            d["password"] = self.password
            d["address"] = self.address
            d["gateway"] = self.gateway
            d["netmask"] = self.netmask
            d["dns"] = self.dns
            d["signal"] = self.signal
        d["result"] = self.result
        d["message"] = self.message
        return d
