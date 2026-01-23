import json
from fastapi.responses import JSONResponse
from rb_flat_buffers.IPC.Request_Network_ConnectWifi import Request_Network_ConnectWifiT
from rb_flat_buffers.IPC.Request_Network_GetWifiList import Request_Network_GetWifiListT
from rb_flat_buffers.IPC.Request_Network_SetNetwork import Request_Network_SetNetworkT
from rb_flat_buffers.IPC.Response_Network_ConnectWifi import Response_Network_ConnectWifiT
from rb_flat_buffers.IPC.Response_Network_GetNetwork import Response_Network_GetNetworkT
from rb_flat_buffers.IPC.Response_Network_GetWifiList import Response_Network_GetWifiListT
from rb_flat_buffers.IPC.Response_Network_SetNetwork import Response_Network_SetNetworkT
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_zenoh.client import ZenohClient

from app.features.network.network_schema import (
    Request_Network_ConnectWifi,
    Request_Network_SetPD,
    Response_Network_ConnectWifi,
    Response_Network_GetWifiList,
    Response_Network_SetPD,
)

zenoh_client = ZenohClient()

class NetworkService(BaseService):
    """
    Network Service
    """

    async def get_network(self):
        """
        [현재 네트워크 조회(이더넷,와이파이,블루투스)]
        """

        # 1) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "network/current",
            flatbuffer_res_T_class=Response_Network_GetNetworkT,
            flatbuffer_buf_size=125,
            timeout=30
        )

        # 2) 에러 확인 및 반환
        if result.get("err") is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[network_module] getNetwork Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"]}
            )

        # 3) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})

    async def set_network(self, request: Request_Network_SetPD) -> Response_Network_SetPD:
        """
        [네트워크 설정]
        """

        # 1) request 객체 생성
        req = Request_Network_SetNetworkT()
        req.ssid = request.ssid
        req.dhcp = request.dhcp
        req.address = request.address
        req.gateway = request.gateway
        req.netmask = request.netmask
        req.dns = request.dns

        # 2) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "network/set",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Network_SetNetworkT,
            flatbuffer_buf_size=125,
            timeout=30
        )

        # 3) 에러 확인 및 반환
        if result["err"] is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[network_module] setNetwork Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"], "request": request.model_dump()}
            )

        # 4) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})

    async def connect_wifi(self, request: Request_Network_ConnectWifi) -> Response_Network_ConnectWifi:
        """
        [와이파이 접속]
        """

        # 1) request 객체 생성
        req = Request_Network_ConnectWifiT()
        req.ssid = request.ssid
        req.password = request.password

        # 2) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "network/wifi/connect",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Network_ConnectWifiT,
            flatbuffer_buf_size=125,
            timeout=30
        )

        # 3) 에러 확인 및 반환
        if result["err"] is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[network_module] connectWifi Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"], "request": request.model_dump()}
            )

        # 4) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})



    async def get_wifi_list(self, rescan: bool = False) -> Response_Network_GetWifiList:
        """
        [와이파이 목록 조회]
        """

        # 1) request 객체 생성
        req = Request_Network_GetWifiListT()
        req.rescan = rescan

        # 2) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "network/wifi/list",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Network_GetWifiListT,
            flatbuffer_buf_size=125,
            timeout=30
        )
        # 3) 에러 확인 및 반환
        if result["err"] is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[network_module] connectWifi Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"], "request": {"rescan": rescan}}
            )

        # 4) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})
