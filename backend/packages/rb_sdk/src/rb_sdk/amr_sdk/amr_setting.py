
from rb_flat_buffers.SLAMNAV.RequestGetRobotType import RequestGetRobotTypeT
from rb_flat_buffers.SLAMNAV.ResponseGetRobotType import ResponseGetRobotTypeT
from rb_flat_buffers.SLAMNAV.RequestSetRobotType import RequestSetRobotTypeT
from rb_flat_buffers.SLAMNAV.ResponseSetRobotType import ResponseSetRobotTypeT
from rb_flat_buffers.SLAMNAV.SettingParam import SettingParamT
from rb_flat_buffers.SLAMNAV.RequestGetPduParam import RequestGetPduParamT
from rb_flat_buffers.SLAMNAV.ResponseGetPduParam import ResponseGetPduParamT
from rb_flat_buffers.SLAMNAV.RequestSetPduParam import RequestSetPduParamT
from rb_flat_buffers.SLAMNAV.ResponseSetPduParam import ResponseSetPduParamT
from rb_flat_buffers.SLAMNAV.RequestGetDriveParam import RequestGetDriveParamT
from rb_flat_buffers.SLAMNAV.ResponseGetDriveParam import ResponseGetDriveParamT

from pydantic import BaseModel
from ..base import RBBaseSDK
from .schema.amr_setting_schema import SlamnavSettingPort

class SettingParam(BaseModel):
    key: str
    type: str
    value: str

class RBAmrSettingSDK(RBBaseSDK,SlamnavSettingPort):
    """Rainbow Robotics AMR Setting SDK"""

    async def get_robot_type(self, robot_model: str, req_id: str) -> ResponseGetRobotTypeT:
        """
        [로봇 타입 조회]
        - req_id: 요청 ID
        - ResponseGetRobotTypeT 객체 반환
        """

        # 1) RequestGetRobotTypeT 객체 생성
        req = RequestGetRobotTypeT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/setting/getRobotType",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetRobotTypeT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Get Robot Type failed: obj_payload is None")

        return result["obj_payload"]

    async def set_robot_type(self, robot_model: str, req_id: str, robot_type: str) -> ResponseSetRobotTypeT:
        """
        [로봇 타입 설정]
        - req_id: 요청 ID
        - robot_type: 로봇 타입
        - ResponseSetRobotTypeT 객체 반환
        """

        # 1) RequestSetRobotTypeT 객체 생성
        req = RequestSetRobotTypeT()
        req.id = req_id
        req.robot_type = robot_type

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/setting/setRobotType",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetRobotTypeT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Set Robot Type failed: obj_payload is None")

        return result["obj_payload"]

    async def get_pdu_param(self, robot_model: str, req_id: str) -> ResponseGetPduParamT:
        """
        [PDU 파라미터 조회]
        - req_id: 요청 ID
        - ResponseGetPduParamT 객체 반환
        """

        # 1) RequestGetPduParamT 객체 생성
        req = RequestGetPduParamT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/setting/getPduParam",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetPduParamT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Get PDU Param failed: obj_payload is None")

        return result["obj_payload"]

    async def set_pdu_param(self, robot_model: str, req_id: str, params: list[SettingParam]) -> ResponseSetPduParamT:
        """
        [PDU 파라미터 설정]
        - req_id: 요청 ID
        - params: PDU 파라미터 목록
          - name: 파라미터 이름
          - value: 파라미터 값
        - ResponseSetPduParamT 객체 반환
        """

        # 1) RequestSetPduParamT 객체 생성
        req = RequestSetPduParamT()
        req.id = req_id
        req.params = []
        for param in params:
            req.params.append(SettingParamT(key=param.key, type=param.type, value=param.value))

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/setting/setPduParam",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetPduParamT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Set PDU Param failed: obj_payload is None")

        return result["obj_payload"]

    async def get_drive_param(self, robot_model: str, req_id: str) -> ResponseGetDriveParamT:
        """
        [PDU 드라이브 파라미터 조회]
        - req_id: 요청 ID
        - ResponsePduDriveParamT 객체 반환
        """

        # 1) RequestGetDriveParamT 객체 생성
        req = RequestGetDriveParamT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/setting/getDriveParam",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetDriveParamT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Get PDU Param failed: obj_payload is None")

        return result["obj_payload"]

# config.json 관련은 일단 pass
# sensor 관련은 일단 pass
    # async def get_sensor_info(self, robot_model: str, req_id: str, target: str) -> Response_Get_Sensor_IndexT:
    #     """
    #     [Get Sensor Index 전송]
    #     - model: SettingRequestModel
    #     - Response_Get_Sensor_IndexT 객체 반환
    #     """
    #     # 1) Request_Get_Sensor_IndexT 객체 생성
    #     req = Request_Get_Sensor_IndexT()
    #     req.id = req_id
    #     req.target = target
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/setting/get_sensor_index",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Get_Sensor_IndexT,
    #         flatbuffer_buf_size=125,
    #     )
    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Get Sensor Index failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def set_sensor_index(self, robot_model: str, req_id: str, target: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_IndexT:
    #     """
    #     [Set Sensor Index 전송]
    #     - model: SettingRequestModel
    #     - Response_Set_Sensor_IndexT 객체 반환
    #     """
    #     # 1) Request_Set_Sensor_IndexT 객체 생성
    #     req = Request_Set_Sensor_IndexT()
    #     req.id = req_id
    #     req.target = target
    #     req.index = index
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/setting/set_sensor_index",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Set_Sensor_IndexT,
    #         flatbuffer_buf_size=125,
    #     )
    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Set Sensor Index failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def set_sensor_on(self, robot_model: str, req_id: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_OnT:
    #     """
    #     [Set Sensor On 전송]
    #     - model: SettingRequestModel
    #     - Response_Set_Sensor_OnT 객체 반환
    #     """
    #     # 1) Request_Set_Sensor_OnT 객체 생성
    #     req = Request_Set_Sensor_OnT()
    #     req.id = req_id
    #     req.index = index
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/setting/set_sensor_on",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Set_Sensor_OnT,
    #         flatbuffer_buf_size=125,
    #     )
    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Set Sensor On failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def get_sensor_off(self, robot_model: str, req_id: str, index: list[Sensor_InfoT]) -> Response_Get_Sensor_OffT:
    #     """
    #     [Get Sensor Off 전송]
    #     - model: SettingRequestModel
    #     - Response_Get_Sensor_OffT 객체 반환
    #     """
    #     # 1) Request_Get_Sensor_OffT 객체 생성
    #     req = Request_Get_Sensor_OffT()
    #     req.id = req_id
    #     req.index = index
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/setting/get_sensor_off",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Get_Sensor_OffT,
    #         flatbuffer_buf_size=125,
    #     )
    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Get Sensor Off failed: obj_payload is None")

    #     return result["obj_payload"]
