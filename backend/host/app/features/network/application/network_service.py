"""
[Email 서비스]
"""

from contextlib import suppress
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
from rb_database import mongo_db
from rb_flat_buffers.IPC import Response_Network_GetNetwork
from rb_modules.log import rb_log
from rb_utils.service_exception import ServiceException
from app.features.network.domain.network import NetworkCommandEnum, NetworkModel
from app.features.network.adapter.output.nmcli_adapter import NetworkNmcliAdapter
from app.features.network.schema.network_dto import Request_Network_ConnectWifi, Request_Network_GetWifiList, Response_Network_GetWifiList, Response_Network_SetPD, Request_Network_SetPD, Response_Network_ConnectWifi


class NetworkService:
    """
    [Network 서비스]
    """
    def __init__(self):
        self.name = "network"
        self.network_port = NetworkNmcliAdapter()

    async def get_network(self):
        """
        [현재 네트워크 조회(이더넷,와이파이,블루투스)]
        """
        try:
            result = await self.network_port.get_network()
            return Response_Network_GetNetwork.Response_Network_GetNetworkT(
                ethernet=result.ethernet,
                wifi=result.wifi,
                bluetooth=result.bluetooth,
            )
        except ServiceException as e:
            rb_log.error(f"[network_service] getNetwork ServiceException : {e.message} {e.status_code}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message}))

    async def set_network(self, request: Request_Network_SetPD) -> Response_Network_SetPD:
        """
        [네트워크 설정]
        """
        # 1) networkModel 객체 생성
        model = NetworkModel()
        model.set_network(request)
        try:
            rb_log.info(f"[network_service] setNetwork : {request.model_dump()}")


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
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[network_service] setNetwork ServiceException : {e.message} {e.status_code}")

            model.result = "fail"
            model.message = str(e.message)

            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def connect_wifi(self, request: Request_Network_ConnectWifi) -> Response_Network_ConnectWifi:
        """
        [와이파이 연결]
        """

        # 1) networkModel 객체 생성
        model = NetworkModel()
        model.set_connect_wifi(request)

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
            print("RESPONSE: ", model.to_dict())
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[network_service] connectWifi ServiceException : {e.message} {e.status_code}")
            model.result = "fail"
            model.message = str(e.message)
            with suppress(Exception):
                mongo_db.db[self.name].update_one({"id": model.id}, {"$set": model.to_dict()})
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def get_wifi_list(self, rescan: bool = False) -> Response_Network_GetWifiList:
        """
        [네트워크 설정]
        """
        try:
            rb_log.info(f"[network_service] getWifiList : {rescan}")

            # 4) 네트워크 검사 실행(어댑터)
            result = await self.network_port.get_wifi_list(rescan)

            # 6) 결과 반환
            return { "list": result, "result": "success", "message": "와이파이 목록 조회가 완료되었습니다." }
        except ServiceException as e:
            rb_log.error(f"[network_service] getWifiList ServiceException : {e.message} {e.status_code}")

            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message}))
