"""
[Email 서비스]
"""

from contextlib import suppress
import platform
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
from rb_database import mongo_db
from rb_flat_buffers.IPC.Network import NetworkT
from rb_flat_buffers.IPC.Request_Network_ConnectWifi import Request_Network_ConnectWifiT
from rb_flat_buffers.IPC.Request_Network_GetWifiList import Request_Network_GetWifiListT
from rb_flat_buffers.IPC.Request_Network_SetNetwork import Request_Network_SetNetworkT
from rb_flat_buffers.IPC.Response_Network_ConnectWifi import Response_Network_ConnectWifiT
from rb_flat_buffers.IPC.Response_Network_GetNetwork import Response_Network_GetNetworkT
from rb_flat_buffers.IPC.Response_Network_GetWifiList import Response_Network_GetWifiListT
from rb_flat_buffers.IPC.Response_Network_SetNetwork import Response_Network_SetNetworkT
from rb_modules.log import rb_log
from rb_utils.parser import t_to_dict
from rb_utils.service_exception import ServiceException

from .adapter.network_linux_adapter import NetworkLinuxAdapter
from .adapter.network_windows_adapter import NetworkWindowsAdapter
from .adapter.network_mac_adapter import NetworkMacAdapter
from .domain.network import NetworkModel,Network

class NetworkService:
    """
    [Network 서비스]
    """
    def __init__(self):
        self.name = "network"
        self.os = platform.system()
        if self.os.lower().__contains__("linux"):
            rb_log.info("[network_service] Host OS : Linux")
            self.network_port = NetworkLinuxAdapter()
        elif self.os.lower().__contains__("windows"):
            self.network_port = NetworkWindowsAdapter()
            rb_log.info("[network_service] Host OS : Windows")
        elif self.os.lower().__contains__("darwin"):
            rb_log.info("[network_service] Host OS : Darwin")
            self.network_port = NetworkMacAdapter()
        else:
            rb_log.error(f"[network_service] Unsupported OS: {self.os}")
            raise ValueError(f"Unsupported OS: {self.os}")

    def network_to_networkT(self, network: Network) -> NetworkT:
        """
        [Network 모델을 Response_Network_GetNetworkT 변환]
        """
        if(network is None):
            return None
        resp_table = NetworkT()
        resp_table.device = network.device if network is not None else None
        resp_table.dhcp = network.dhcp if network is not None else None
        resp_table.dns = network.dns if network is not None else None
        resp_table.ssid = network.ssid if network is not None else None
        resp_table.address = network.address if network is not None else None
        resp_table.netmask = network.netmask if network is not None else None
        resp_table.gateway = network.gateway if network is not None else None
        resp_table.signal = network.signal if network is not None else None
        return resp_table

    async def get_network(self) -> Response_Network_GetNetworkT:
        """
        [현재 네트워크 조회(이더넷,와이파이,블루투스)]
        """
        try:
            result = await self.network_port.get_network()
            print("result: ", result, flush=True)
            resp_table = Response_Network_GetNetworkT()
            resp_table.ethernet = self.network_to_networkT(result.get("ethernet"))
            resp_table.wifi = self.network_to_networkT(result.get("wifi"))
            resp_table.bluetooth = self.network_to_networkT(result.get("bluetooth"))

            print("resp_table: ", resp_table, flush=True)
            return resp_table

        except ServiceException as e:
            rb_log.error(f"[network_service] getNetwork ServiceException : {e.message} {e.status_code}")
            raise e

    async def set_network(self, request: Request_Network_SetNetworkT) -> Response_Network_SetNetworkT:
        """
        [네트워크 설정]
        """

        # 0) request 파싱
        req_data = t_to_dict(request)

        # 1) networkModel 객체 생성
        model = NetworkModel()
        model.set_network(req_data)
        try:
            rb_log.info(f"[network_service] setNetwork : {req_data}")


            # 2) DB 저장
            with suppress(Exception):
                mongo_db.db[self.name].insert_one(model.to_dict())

            # 3) 요청 검사
            model.check_variables()

            # 4) 네트워크 검사 실행(어댑터)
            result = await self.network_port.set_network(model)

            # 5) 결과 DB 저장
            model.result = "success"
            model.message = "네트워크 설정이 완료되었습니다."
            model.network_info(result)
            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})

            # 6) 결과 반환
            resp_table = Response_Network_SetNetworkT()
            resp_table.ssid = result.ssid
            resp_table.dhcp = result.dhcp
            resp_table.address = result.address
            resp_table.gateway = result.gateway
            resp_table.netmask = result.netmask
            resp_table.dns = result.dns
            resp_table.result = model.result
            resp_table.message = model.message
            return resp_table

        except ServiceException as e:
            rb_log.error(f"[network_service] setNetwork ServiceException : {e.message} {e.status_code}")

            model.result = "fail"
            model.message = str(e.message)

            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})
            raise e

    async def connect_wifi(self, req: Request_Network_ConnectWifiT) -> Response_Network_ConnectWifiT:
        """
        [와이파이 연결]
        """

        # 0) request 파싱
        req_data = t_to_dict(req)

        # 1) networkModel 객체 생성
        model = NetworkModel()
        model.set_connect_wifi(req_data)

        try:
            rb_log.info(f"[network_service] connectWifi : {model.to_dict()}")

            # 2) DB 저장
            with suppress(Exception):
                mongo_db.db[self.name].insert_one(model.to_dict())

            # 3) 요청 검사
            model.check_variables()

            # 4) 네트워크 검사 실행(어댑터)
            result = await self.network_port.connect_wifi(model)
            # 5) 결과 DB 저장
            model.result = "success"
            model.message = "와이파이 연결이 완료되었습니다."
            model.network_info(result)
            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})

            # 6) 결과 반환
            resp_table = Response_Network_ConnectWifiT()
            resp_table.ssid = result.ssid
            resp_table.address = result.address
            resp_table.gateway = result.gateway
            resp_table.netmask = result.netmask
            resp_table.dns = result.dns
            resp_table.signal = result.signal
            resp_table.result = model.result
            resp_table.message = model.message
            return resp_table
        except ServiceException as e:
            rb_log.error(f"[network_service] connectWifi ServiceException : {e.message} {e.status_code}")
            model.result = "fail"
            model.message = str(e.message)
            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})
            raise e

    async def get_wifi_list(self, request: Request_Network_GetWifiListT) -> Response_Network_GetWifiListT:
        """
        [네트워크 설정]
        """
        try:
            req_data = t_to_dict(request)

            rb_log.info(f"[network_service] getWifiList : {req_data}")

            # 4) 네트워크 검사 실행(어댑터)
            result = await self.network_port.get_wifi_list(req_data)

            resp_table = Response_Network_GetWifiListT()
            resp_table.list = result
            resp_table.result = "success"
            resp_table.message = "와이파이 목록 조회가 완료되었습니다."

            # 6) 결과 반환
            return resp_table
        except ServiceException as e:
            rb_log.error(f"[network_service] getWifiList ServiceException : {e.message} {e.status_code}")

            raise e
