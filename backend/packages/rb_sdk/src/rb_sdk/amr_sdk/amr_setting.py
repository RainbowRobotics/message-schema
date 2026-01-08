
from rb_flat_buffers.SLAMNAV.Request_Get_Pdu_Param import Request_Get_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Request_Get_Robot_Type import Request_Get_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Request_Get_Sensor_Index import Request_Get_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Request_Get_Sensor_Off import Request_Get_Sensor_OffT
from rb_flat_buffers.SLAMNAV.Request_Set_Pdu_Param import Request_Set_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Request_Set_Robot_Type import Request_Set_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Request_Set_Sensor_Index import Request_Set_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Request_Set_Sensor_On import Request_Set_Sensor_OnT
from rb_flat_buffers.SLAMNAV.Response_Get_Pdu_Param import Response_Get_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Response_Get_Robot_Type import Response_Get_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Response_Get_Sensor_Index import Response_Get_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Response_Get_Sensor_Off import Response_Get_Sensor_OffT
from rb_flat_buffers.SLAMNAV.Response_Set_Pdu_Param import Response_Set_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Response_Set_Robot_Type import Response_Set_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Response_Set_Sensor_Index import Response_Set_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Response_Set_Sensor_On import Response_Set_Sensor_OnT
from rb_flat_buffers.SLAMNAV.Sensor_Info import Sensor_InfoT
from rb_flat_buffers.SLAMNAV.Setting_Param import Setting_ParamT
from rb_zenoh.client import ZenohClient

from .schema.amr_setting_schema import SlamnavSettingPort


class RBAmrSettingSDK(SlamnavSettingPort):
    """Rainbow Robotics AMR Setting SDK"""
    client: ZenohClient
    def __init__(self, client: ZenohClient):
        self.client = client

    async def get_robot_type(self, robot_model: str, req_id: str) -> Response_Get_Robot_TypeT:
        """
        [Get Robot Type 전송]
        - model: SettingRequestModel
        - Response_Get_Robot_TypeT 객체 반환
        """
        # 1) Request_Get_Robot_TypeT 객체 생성
        req = Request_Get_Robot_TypeT()
        req.id = req_id
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/get_robot_type",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Robot_TypeT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def set_robot_type(self, robot_model: str, req_id: str, robot_type: str) -> Response_Set_Robot_TypeT:
        """
        [Set Robot Type 전송]
        - model: SettingRequestModel
        - Response_Set_Robot_TypeT 객체 반환
        """
        # 1) Request_Set_Robot_TypeT 객체 생성
        req = Request_Set_Robot_TypeT()
        req.id = req_id
        req.robot_type = robot_type
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/set_robot_type",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Set_Robot_TypeT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def get_sensor_index(self, robot_model: str, req_id: str, target: str) -> Response_Get_Sensor_IndexT:
        """
        [Get Sensor Index 전송]
        - model: SettingRequestModel
        - Response_Get_Sensor_IndexT 객체 반환
        """
        # 1) Request_Get_Sensor_IndexT 객체 생성
        req = Request_Get_Sensor_IndexT()
        req.id = req_id
        req.target = target
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/get_sensor_index",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Sensor_IndexT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def set_sensor_index(self, robot_model: str, req_id: str, target: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_IndexT:
        """
        [Set Sensor Index 전송]
        - model: SettingRequestModel
        - Response_Set_Sensor_IndexT 객체 반환
        """
        # 1) Request_Set_Sensor_IndexT 객체 생성
        req = Request_Set_Sensor_IndexT()
        req.id = req_id
        req.target = target
        req.index = index
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/set_sensor_index",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Set_Sensor_IndexT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def set_sensor_on(self, robot_model: str, req_id: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_OnT:
        """
        [Set Sensor On 전송]
        - model: SettingRequestModel
        - Response_Set_Sensor_OnT 객체 반환
        """
        # 1) Request_Set_Sensor_OnT 객체 생성
        req = Request_Set_Sensor_OnT()
        req.id = req_id
        req.index = index
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/set_sensor_on",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Set_Sensor_OnT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def get_sensor_off(self, robot_model: str, req_id: str, index: list[Sensor_InfoT]) -> Response_Get_Sensor_OffT:
        """
        [Get Sensor Off 전송]
        - model: SettingRequestModel
        - Response_Get_Sensor_OffT 객체 반환
        """
        # 1) Request_Get_Sensor_OffT 객체 생성
        req = Request_Get_Sensor_OffT()
        req.id = req_id
        req.index = index
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/get_sensor_off",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Sensor_OffT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def get_pdu_param(self, robot_model: str, req_id: str) -> Response_Get_Pdu_ParamT:
        """
        [Get PDU Param 전송]
        - model: SettingRequestModel
        - Response_Get_Pdu_ParamT 객체 반환
        """
        # 1) Request_Get_Pdu_ParamT 객체 생성
        req = Request_Get_Pdu_ParamT()
        req.id = req_id
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/get_pdu_param",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Pdu_ParamT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def set_pdu_param(self, robot_model: str, req_id: str, params: list[Setting_ParamT]) -> Response_Set_Pdu_ParamT:
        """
        [Set PDU Param 전송]
        - model: SettingRequestModel
        - Response_Set_Pdu_ParamT 객체 반환
        """
        # 1) Request_Set_Pdu_ParamT 객체 생성
        req = Request_Set_Pdu_ParamT()
        req.id = req_id
        req.params = params
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/setting/set_pdu_param",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Set_Pdu_ParamT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        return result["dict_payload"]
