from app.features.network.port.network_port import NetworkPort
from app.features.network.domain.network import NetworkModel


class NetworkMacAdapter(NetworkPort):
    """
    [Network Mac 어댑터]
    """
    async def get_network(self) -> dict:
        """
        [네트워크 조회]
        """
        raise NotImplementedError("Not implemented")
    async def get_ethernet(self) -> dict:
        raise NotImplementedError("Not implemented")
    async def get_wifi(self) -> dict:
        raise NotImplementedError("Not implemented")
    async def get_bluetooth(self) -> dict:
        raise NotImplementedError("Not implemented")
    async def set_network(self, model: NetworkModel) -> dict:
        raise NotImplementedError("Not implemented")
    async def connect_wifi(self, model: NetworkModel) -> dict:
        raise NotImplementedError("Not implemented")
    async def scan_wifi(self) -> dict:
        raise NotImplementedError("Not implemented")
    async def get_wifi_list(self) -> dict:
        raise NotImplementedError("Not implemented")
