from rb_flat_buffers.SLAMNAV.Request_Localization_AutoInit import Request_Localization_AutoInitT
from rb_flat_buffers.SLAMNAV.Request_Localization_Init import Request_Localization_InitT
from rb_flat_buffers.SLAMNAV.Request_Localization_RandomInit import Request_Localization_RandomInitT
from rb_flat_buffers.SLAMNAV.Request_Localization_SemiAutoInit import (
    Request_Localization_SemiAutoInitT,
)
from rb_flat_buffers.SLAMNAV.Request_Localization_Start import Request_Localization_StartT
from rb_flat_buffers.SLAMNAV.Request_Localization_Stop import Request_Localization_StopT
from rb_flat_buffers.SLAMNAV.Response_Localization_AutoInit import Response_Localization_AutoInitT
from rb_flat_buffers.SLAMNAV.Response_Localization_Init import Response_Localization_InitT
from rb_flat_buffers.SLAMNAV.Response_Localization_RandomInit import (
    Response_Localization_RandomInitT,
)
from rb_flat_buffers.SLAMNAV.Response_Localization_SemiAutoInit import (
    Response_Localization_SemiAutoInitT,
)
from rb_flat_buffers.SLAMNAV.Response_Localization_Start import Response_Localization_StartT
from rb_flat_buffers.SLAMNAV.Response_Localization_Stop import Response_Localization_StopT
from rb_zenoh.client import ZenohClient

from .schema.amr_localization_schema import SlamnavLocalizationPort


class RBAmrLocalizationSDK(SlamnavLocalizationPort):
    """Rainbow Robotics AMR Localization SDK"""
    client: ZenohClient
    def __init__(self, client: ZenohClient):
        self.client = client

    async def localization_init(self, robot_model: str, req_id: str, x: float, y: float, z: float, rz: float) -> Response_Localization_InitT:
        """
        [Localization Init 전송]
        - model: LocalizationRequestModel
        - Response_Localization_InitT 객체 반환
        """
        # 1) Request_Localization_InitT 객체 생성
        req = Request_Localization_InitT()
        req.id = req_id
        req.x = x
        req.y = y
        req.z = z
        req.rz = rz

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/init",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_InitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def localization_semi_auto_init(self, robot_model: str, req_id: str) -> Response_Localization_SemiAutoInitT:
        """
        [Localization Semi Auto Init 전송]
        - model: LocalizationRequestModel
        - Response_Localization_SemiAutoInitT 객체 반환
        """
        # 1) Request_Localization_SemiAutoInitT 객체 생성
        req = Request_Localization_SemiAutoInitT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/semi_auto_init",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_SemiAutoInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def localization_auto_init(self, robot_model: str, req_id: str) -> Response_Localization_AutoInitT:
        """
        [Localization Auto Init 전송]
        - model: LocalizationRequestModel
        - Response_Localization_AutoInitT 객체 반환
        """
        # 1) Request_Localization_AutoInitT 객체 생성
        req = Request_Localization_AutoInitT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/auto_init",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_AutoInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def localization_start(self, robot_model: str, req_id: str) -> Response_Localization_StartT:
        """
        [Localization Start 전송]
        - model: LocalizationRequestModel
        - Response_Localization_StartT 객체 반환
        """
        # 1) Request_Localization_StartT 객체 생성
        req = Request_Localization_StartT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/start",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_StartT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def localization_stop(self, robot_model: str, req_id: str) -> Response_Localization_StopT:
        """
        [Localization Stop 전송]
        - model: LocalizationRequestModel
        - Response_Localization_StopT 객체 반환
        """
        # 1) Request_Localization_StopT 객체 생성
        req = Request_Localization_StopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_StopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def localization_random_init(self, robot_model: str, req_id: str, random_seed: str) -> Response_Localization_RandomInitT:
        """
        [Localization Random Init 전송]
        - model: LocalizationRequestModel
        - Response_Localization_RandomInitT 객체 반환
        """
        # 1) Request_Localization_RandomInitT 객체 생성
        req = Request_Localization_RandomInitT()
        req.id = req_id
        req.random_seed = random_seed

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/localization/random_init",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Localization_RandomInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]
