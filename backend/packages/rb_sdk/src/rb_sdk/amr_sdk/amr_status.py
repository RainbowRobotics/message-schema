from typing import Any
from rb_sdk.base import RBBaseSDK
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.GlobalPath import GlobalPathT
from rb_flat_buffers.SLAMNAV.LocalPath import LocalPathT

class RBAmrStatusSDK(RBBaseSDK):
    """Rainbow Robotics AMR Status SDK"""

    async def get_status(self, robot_model: str, robot_id: str) -> dict[str, Any]:
        """
        [상태 조회]
        - req_id: 요청 ID
        - ResponseStatusT 객체 반환
        """
        _, _, obj, _ = await self.zenoh_client.receive_one(
            f"amr/{robot_model}/{robot_id}/status",
            flatbuffer_obj_t=StatusT
        )

        if obj is None:
            raise RuntimeError("Call Get Status failed: obj_payload is None")

        return obj

    async def get_move_status(self, robot_model: str, robot_id: str) -> dict[str, Any]:
        """
        [이동 상태 조회]
        - req_id: 요청 ID
        - ResponseMoveStatusT 객체 반환
        """
        _, _, obj, _ = await self.zenoh_client.receive_one(
            f"amr/{robot_model}/{robot_id}/moveStatus",
            flatbuffer_obj_t=MoveStatusT
        )

        if obj is None:
            raise RuntimeError("Call Get Move Status failed: obj_payload is None")

        return obj

    async def get_lidar2d(self, robot_model: str, robot_id: str) -> dict[str, Any]:
        """
        [라이다 2D 조회]
        - req_id: 요청 ID
        - ResponseLidar2D 객체 반환
        """
        _, _, obj, _ = await self.zenoh_client.receive_one(
            f"amr/{robot_model}/{robot_id}/lidar2d",
            flatbuffer_obj_t=Lidar2DT
        )

        if obj is None:
            raise RuntimeError("Call Get Lidar2D failed: obj_payload is None")

        return obj

    async def get_global_path(self, robot_model: str, robot_id: str) -> dict[str, Any]:
        """
        [전역 경로 조회]
        - req_id: 요청 ID
        - ResponseGlobalPath 객체 반환
        """
        _, _, obj, _ = await self.zenoh_client.receive_one(
            f"amr/{robot_model}/{robot_id}/globalPath",
            flatbuffer_obj_t=GlobalPathT
        )

        if obj is None:
            raise RuntimeError("Call Get Global Path failed: obj_payload is None")

        return obj

    async def get_local_path(self, robot_model: str, robot_id: str) -> dict[str, Any]:
        """
        [지역 경로 조회]
        - req_id: 요청 ID
        - ResponseLocalPath 객체 반환
        """
        _, _, obj, _ = await self.zenoh_client.receive_one(
            f"amr/{robot_model}/{robot_id}/localPath",
            flatbuffer_obj_t=LocalPathT
        )

        if obj is None:
            raise RuntimeError("Call Get Local Path failed: obj_payload is None")

        return obj
