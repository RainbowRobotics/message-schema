import rb_influxdb.influxdb_client as influxdb_client
from rb_flat_buffers.SLAMNAV.GlobalPath import GlobalPathT
from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.LocalPath import LocalPathT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_influxdb import flatbuffer_to_point

from app.socket.socket_client import socket_client


class AmrStatusService:
    """
    AMR Status Service
    """
    def __init__(self):
        self.status: dict[str, StatusT] = {}
        self.move_status: dict[str, MoveStatusT] = {}
        self.lidar2d: dict[str, Lidar2DT] = {}
        self.global_path: dict[str, GlobalPathT] = {}
        self.local_path: dict[str, LocalPathT] = {}

    async def set_status(self, topic: str, obj: StatusT):
        """ zenoh Status -> socket Status """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        self.status[f"{robot_model}/{robot_id}"] = obj

        # print("Status Set : ", self.get_status(robot_model, robot_id), flush=True)
        # 소켓 클라이언트로 전송
        await socket_client.emit(topic, obj)

        # common으로 전송
        # await status_zenoh_router.publish(f"?/status", obj)

        # influxdb 저장
        # try:
        #     # StatusT를 자동으로 InfluxDB Point로 변환
        #     # point = flatbuffer_to_point(
        #     #     obj=obj,
        #     #     measurement="status",
        #     #     tags={
        #     #         "robot_model": robot_model,
        #     #         "robot_id": robot_id,
        #     #     },
        #     # )

        #     # influxdb_client.write_point(
        #     #     org="rrs",
        #     #     bucket="amr-status",
        #     #     point=point,
        #     # )

        # except Exception as e:
        #     print(f"InfluxDB 저장 실패: {e}", flush=True)

    def get_status(self, robot_model: str, robot_id: str) -> StatusT:
        """ API에서 요청하면 마지막 status 반환 """
        return self.status.get(f"{robot_model}/{robot_id}")

    async def set_move_status(self, topic: str, obj: MoveStatusT):
        """ zenoh MoveStatus -> socket MoveStatus """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]

        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        self.move_status[f"{robot_model}/{robot_id}"] = obj

        # print("MoveStatus Sub : ", robot_model, robot_id, topic, obj, flush=True)

        await socket_client.emit(topic, obj)

        # common으로 전송
        # await status_zenoh_router.publish(f"?/moveStatus", obj)

    def get_move_status(self, robot_model: str, robot_id: str) -> MoveStatusT:
        """ API에서 요청하면 마지막 moveStatus 반환 """
        return self.move_status.get(f"{robot_model}/{robot_id}")

    async def set_lidar2d(self, topic: str, obj: Lidar2DT):
        """ zenoh Lidar2D -> socket Lidar2D """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        self.lidar2d[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

        # common으로 전송
        # await status_zenoh_router.publish(f"?/lidar2d", obj)

    def get_lidar2d(self, robot_model: str, robot_id: str) -> Lidar2DT:
        """ API에서 요청하면 마지막 lidar2d 반환 """
        return self.lidar2d.get(f"{robot_model}/{robot_id}")

    async def set_global_path(self, topic: str, obj: GlobalPathT):
        """ zenoh GlobalPath -> socket GlobalPath """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        self.global_path[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

    def get_global_path(self, robot_model: str, robot_id: str) -> GlobalPathT:
        """ API에서 요청하면 마지막 globalPath 반환 """
        return self.global_path.get(f"{robot_model}/{robot_id}")

    async def set_local_path(self, topic: str, obj: LocalPathT):
        """ zenoh LocalPath -> socket LocalPath """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        self.local_path[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

    def get_local_path(self, robot_model: str, robot_id: str) -> LocalPathT:
        """ API에서 요청하면 마지막 localPath 반환 """
        return self.local_path.get(f"{robot_model}/{robot_id}")
