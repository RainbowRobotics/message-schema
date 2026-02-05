from rb_flat_buffers.SLAMNAV.RequestLocalizationAutoInit import RequestLocalizationAutoInitT
from rb_flat_buffers.SLAMNAV.RequestLocalizationInit import RequestLocalizationInitT
from rb_flat_buffers.SLAMNAV.RequestLocalizationRandomInit import RequestLocalizationRandomInitT
from rb_flat_buffers.SLAMNAV.RequestLocalizationSemiAutoInit import RequestLocalizationSemiAutoInitT
from rb_flat_buffers.SLAMNAV.RequestLocalizationStart import RequestLocalizationStartT
from rb_flat_buffers.SLAMNAV.RequestLocalizationStop import RequestLocalizationStopT
from rb_flat_buffers.SLAMNAV.ResponseLocalizationAutoInit import ResponseLocalizationAutoInitT
from rb_flat_buffers.SLAMNAV.ResponseLocalizationInit import ResponseLocalizationInitT
from rb_flat_buffers.SLAMNAV.ResponseLocalizationRandomInit import ResponseLocalizationRandomInitT
from rb_flat_buffers.SLAMNAV.ResponseLocalizationSemiAutoInit import (
    ResponseLocalizationSemiAutoInitT,
)
from rb_flat_buffers.SLAMNAV.ResponseLocalizationStart import ResponseLocalizationStartT
from rb_flat_buffers.SLAMNAV.ResponseLocalizationStop import ResponseLocalizationStopT

from ..base import RBBaseSDK


class RBAmrLocalizationSDK(RBBaseSDK):
    """Rainbow Robotics AMR Localization SDK"""

    async def localization_init(self, robot_model: str, req_id: str, x: float, y: float, z: float, rz: float) -> ResponseLocalizationInitT:
        """
        [위치 초기화]
        - req_id: 요청 ID
        - x: x 좌표
        - y: y 좌표
        - z: z 좌표
        - rz: rz 좌표
        - ResponseLocalizationInitT 객체 반환
        """

        # 1) RequestLocalizationInitT 객체 생성
        req = RequestLocalizationInitT()
        req.id = req_id
        req.pose.x = x
        req.pose.y = y
        req.pose.z = z
        req.pose.rz = rz

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/init",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Semi Auto Init failed: obj_payload is None")

        return result["obj_payload"]

    async def localization_random_init(self, robot_model: str, req_id: str) -> ResponseLocalizationRandomInitT:
        """
        [위치 초기화 (랜덤)]
        - req_id: 요청 ID
        - ResponseLocalizationRandomInitT 객체 반환
        """

        # 1) RequestLocalizationRandomInitT 객체 생성
        req = RequestLocalizationRandomInitT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/randomInit",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationRandomInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Random Init failed: obj_payload is None")

        return result["obj_payload"]

    async def localization_auto_init(self, robot_model: str, req_id: str) -> ResponseLocalizationAutoInitT:
        """
        [위치 초기화 (자동)]
        - req_id: 요청 ID
        - ResponseLocalizationAutoInitT 객체 반환
        """

        # 1) RequestLocalizationAutoInitT 객체 생성
        req = RequestLocalizationAutoInitT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/autoInit",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationAutoInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Start failed: obj_payload is None")

        return result["obj_payload"]

    async def localization_semi_auto_init(self, robot_model: str, req_id: str) -> ResponseLocalizationSemiAutoInitT:
        """
        [위치 초기화 (반자동)]
        - req_id: 요청 ID
        - ResponseLocalizationSemiAutoInitT 객체 반환
        """

        # 1) RequestLocalizationSemiAutoInitT 객체 생성
        req = RequestLocalizationSemiAutoInitT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/semiAutoInit",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationSemiAutoInitT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Auto Init failed: obj_payload is None")

        return result["obj_payload"]

    async def localization_start(self, robot_model: str, req_id: str) -> ResponseLocalizationStartT:
        """
        [위치추정 시작]
        - req_id: 요청 ID
        - ResponseLocalizationStartT 객체 반환
        """

        # 1) RequestLocalizationStartT 객체 생성
        req = RequestLocalizationStartT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/start",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationStartT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Stop failed: obj_payload is None")

        return result["obj_payload"]

    async def localization_stop(self, robot_model: str, req_id: str) -> ResponseLocalizationStopT:
        """
        [위치추정 중지]
        - req_id: 요청 ID
        - ResponseLocalizationStopT 객체 반환
        """
        # 1) RequestLocalizationStopT 객체 생성
        req = RequestLocalizationStopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/localization/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLocalizationStopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Localization Stop failed: obj_payload is None")

        return result["obj_payload"]
