from rb_flat_buffers.SLAMNAV.GlobalPath import GlobalPathT
from rb_flat_buffers.SLAMNAV.Lidar2D import Lidar2DT
from rb_flat_buffers.SLAMNAV.LocalPath import LocalPathT
from rb_flat_buffers.SLAMNAV.MoveStatus import MoveStatusT
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_influxdb import flatbuffer_to_point, write_point
from rb_sdk.amr import RBAmrSDK
from rb_utils.parser import t_to_dict

from app.socket.socket_client import socket_client


rb_amr_sdk = RBAmrSDK()
class AmrStatusService:
    """
    AMR Status Service
    """

    async def set_status(self, topic: str, obj: StatusT):
        """ zenoh Status -> socket Status """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        # self.status[f"{robot_model}/{robot_id}"] = obj

        # print("Status Set : ", self.get_status(robot_model, robot_id), flush=True)
        # 소켓 클라이언트로 전송
        await socket_client.emit(topic, obj)

        # influxdb 저장
        dict_obj=t_to_dict(obj)
        self.write_influxdb(measurement="condition", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("condition"))
        self.write_influxdb(measurement="imu", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("imu"))
        self.write_influxdb(measurement="motor0", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("motor")[0])
        self.write_influxdb(measurement="motor1", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("motor")[1])
        self.write_influxdb(measurement="power", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("power"))
        self.write_influxdb(measurement="robotState", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("robotState"))
        self.write_influxdb(measurement="robotSafetyIoState", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("robotSafetyIoState"))
        self.write_influxdb(measurement="map", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("map"))


    def write_influxdb(self, measurement: str, robot_model: str, robot_id: str, obj: dict):
        try:
            tags={"robot_model": robot_model, "robot_id": robot_id}
            point = flatbuffer_to_point(obj=obj, tags=tags, measurement=measurement)
            write_point(org="rrs", bucket="rrs-rt", point=point)
        except Exception as e:
            print(f"InfluxDB 저장 실패: {e}", flush=True)

    async def get_status(self, robot_model: str, robot_id: str):
        """ API에서 요청하면 마지막 status 반환 """
        return await rb_amr_sdk.status.get_status(robot_model, robot_id)

    async def set_move_status(self, topic: str, obj: MoveStatusT):
        """ zenoh MoveStatus -> socket MoveStatus """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]

        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        # self.move_status[f"{robot_model}/{robot_id}"] = obj

        # print("MoveStatus Sub : ", robot_model, robot_id, topic, obj, flush=True)

        await socket_client.emit(topic, obj)

        # common으로 전송
        # await status_zenoh_router.publish(f"?/moveStatus", obj)

        # influxdb 저장
        dict_obj=t_to_dict(obj)
        self.write_influxdb(measurement="moveState", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("moveState"))
        self.write_influxdb(measurement="pose", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("pose"))
        self.write_influxdb(measurement="vel", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("vel"))
        self.write_influxdb(measurement="curNode", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("curNode"))
        self.write_influxdb(measurement="goalNode", robot_model=robot_model, robot_id=robot_id, obj=dict_obj.get("goalNode"))

    async def get_move_status(self, robot_model: str, robot_id: str) -> MoveStatusT:
        """ API에서 요청하면 마지막 moveStatus 반환 """
        return await rb_amr_sdk.status.get_move_status(robot_model, robot_id)

    async def set_lidar2d(self, topic: str, obj: Lidar2DT):
        """ zenoh Lidar2D -> socket Lidar2D """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        # self.lidar2d[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

        # common으로 전송
        # await status_zenoh_router.publish(f"?/lidar2d", obj)

    async def get_lidar2d(self, robot_model: str, robot_id: str) -> Lidar2DT:
        """ API에서 요청하면 마지막 lidar2d 반환 """
        return await rb_amr_sdk.status.get_lidar2d(robot_model, robot_id)

    async def set_global_path(self, topic: str, obj: GlobalPathT):
        """ zenoh GlobalPath -> socket GlobalPath """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        # self.global_path[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

    async def get_global_path(self, robot_model: str, robot_id: str) -> GlobalPathT:
        """ API에서 요청하면 마지막 globalPath 반환 """
        return await rb_amr_sdk.status.get_global_path(robot_model, robot_id)

    async def set_local_path(self, topic: str, obj: LocalPathT):
        """ zenoh LocalPath -> socket LocalPath """
        # 로봇 모델과 아이디 추출
        robot_model, robot_id = topic.split("/")[1:3]
        # 로봇 모델과 아이디가 없으면 반환
        if not robot_model or not robot_id:
            return

        # self.local_path[f"{robot_model}/{robot_id}"] = obj

        await socket_client.emit(topic, obj)

    async def get_local_path(self, robot_model: str, robot_id: str) -> LocalPathT:
        """ API에서 요청하면 마지막 localPath 반환 """
        return await rb_amr_sdk.status.get_local_path(robot_model, robot_id)
