from pydantic import BaseModel
from rb_flat_buffers.SLAMNAV.ObsBox import ObsBoxT
from rb_flat_buffers.SLAMNAV.RequestChargeTrigger import RequestChargeTriggerT
from rb_flat_buffers.SLAMNAV.RequestDetectMarker import RequestDetectMarkerT
from rb_flat_buffers.SLAMNAV.RequestDockControl import RequestDockControlT
from rb_flat_buffers.SLAMNAV.RequestGetObsBox import RequestGetObsBoxT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyField import RequestGetSafetyFieldT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyFlag import RequestGetSafetyFlagT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyIo import RequestGetSafetyIoT
from rb_flat_buffers.SLAMNAV.RequestJog import RequestJogT
from rb_flat_buffers.SLAMNAV.RequestLedMode import RequestLedModeT
from rb_flat_buffers.SLAMNAV.RequestMotorMode import RequestMotorModeT
from rb_flat_buffers.SLAMNAV.RequestPathMode import RequestPathModeT
from rb_flat_buffers.SLAMNAV.RequestSensorMode import RequestSensorModeT
from rb_flat_buffers.SLAMNAV.RequestSetObsBox import RequestSetObsBoxT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyField import RequestSetSafetyFieldT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyFlag import RequestSetSafetyFlagT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyIo import RequestSetSafetyIoT
from rb_flat_buffers.SLAMNAV.ResponseChargeTrigger import ResponseChargeTriggerT
from rb_flat_buffers.SLAMNAV.ResponseDetectMarker import ResponseDetectMarkerT
from rb_flat_buffers.SLAMNAV.ResponseDockControl import ResponseDockControlT
from rb_flat_buffers.SLAMNAV.ResponseGetObsBox import ResponseGetObsBoxT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyField import ResponseGetSafetyFieldT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyFlag import ResponseGetSafetyFlagT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyIo import ResponseGetSafetyIoT
from rb_flat_buffers.SLAMNAV.ResponseJog import ResponseJogT
from rb_flat_buffers.SLAMNAV.ResponseLedMode import ResponseLedModeT
from rb_flat_buffers.SLAMNAV.ResponseMotorMode import ResponseMotorModeT
from rb_flat_buffers.SLAMNAV.ResponsePathMode import ResponsePathModeT
from rb_flat_buffers.SLAMNAV.ResponseSensorMode import ResponseSensorModeT
from rb_flat_buffers.SLAMNAV.ResponseSetObsBox import ResponseSetObsBoxT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyField import ResponseSetSafetyFieldT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyFlag import ResponseSetSafetyFlagT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyIo import ResponseSetSafetyIoT
from rb_flat_buffers.SLAMNAV.SafetyFlag import SafetyFlagT

from ..base import RBBaseSDK


class SafetyFlag(BaseModel):
    name: str
    value: bool

class RBAmrControlSDK(RBBaseSDK):
    """Rainbow Robotics AMR Control SDK"""

    async def control_get_safety_field(self, robot_model: str, req_id: str) -> ResponseGetSafetyFieldT:
        """
        [안전 필드 조회]
        - ResponseGetSafetyFieldT 객체 반환
        """

        # 1) RequestGetSafetyFieldT 객체 생성
        req = RequestGetSafetyFieldT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/getSafetyField",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyFieldT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety Field failed: obj_payload is None")

        return result["obj_payload"]

    async def control_set_safety_field(self, robot_model: str, req_id: str, safety_field: int) -> ResponseSetSafetyFieldT:
        """
        [안전 필드 설정]
        - safety_field: 안전 필드 번호
        - ResponseSetSafetyFieldT 객체 반환
        """

        # 1) RequestSetSafetyFieldT 객체 생성
        req = RequestSetSafetyFieldT()
        req.id = req_id
        req.safety_field = safety_field

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/setSafetyField",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyFieldT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety Field failed: obj_payload is None")

        return result["obj_payload"]

    async def control_get_safety_flag(self, robot_model: str, req_id: str) -> ResponseGetSafetyFlagT:
        """
        [안전 플래그 조회]
        - ResponseGetSafetyFlagT 객체 반환
        """

        # 1) RequestGetSafetyFlagT 객체 생성
        req = RequestGetSafetyFlagT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/getSafetyFlag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyFlagT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Reset Safety Flag failed: obj_payload is None")

        return result["obj_payload"]

    async def control_set_safety_flag(self, robot_model: str, req_id: str, reset_flag: list[SafetyFlag]) -> ResponseSetSafetyFlagT:
        """
        [안전 플래그 설정]
        - reset_flag: 안전 플래그 목록
          - key: 안전 플래그 이름 (string)
          - value: 안전 플래그 값 (bool)
        - ResponseSetSafetyFlagT 객체 반환
        """

        # 1) RequestSetSafetyFlagT 객체 생성
        req = RequestSetSafetyFlagT()
        req.id = req_id
        req.resetFlag = []

        for flag in reset_flag:
            safety_flag = SafetyFlagT()
            safety_flag.name = flag.name
            safety_flag.value = flag.value
            req.resetFlag.add(safety_flag)

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/setSafetyFlag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyFlagT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Reset Safety Flag failed: obj_payload is None")

        return result["obj_payload"]

    async def control_get_safety_io(self, robot_model: str, req_id: str) -> ResponseGetSafetyIoT:
        """
        [안전 IO 조회]
        - ResponseGetSafetyIoT 객체 반환
        """

        # 1) RequestGetSafetyIoT 객체 생성
        req = RequestGetSafetyIoT()
        req.id = req_id
        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/getSafetyIo",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyIoT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety IO failed: obj_payload is None")

        return result["obj_payload"]

    async def control_set_safety_io(self, robot_model: str, req_id: str, mcu0_din: list[bool], mcu1_din: list[bool]) -> ResponseSetSafetyIoT:
        """
        [안전 IO 설정]
        - mcu0_din: MCU0 Digital Input (8bit)
        - mcu1_din: MCU1 Digital Input (8bit)
        - ResponseSetSafetyIoT 객체 반환
        """

        # 1) RequestSetSafetyIoT 객체 생성
        req = RequestSetSafetyIoT()
        req.id = req_id
        req.mcu0_din = mcu0_din
        req.mcu1_din = mcu1_din

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/setSafetyIo",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyIoT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety IO failed: obj_payload is None")

        return result["obj_payload"]

    # async def control_random_sequence(self, robot_model: str, req_id: str) -> Response_Random_SequenceT:
    #     """
    #     [Control Random Sequence 전송]
    #     - model: ControlRequestModel
    #     - Response_Random_SequenceT 객체 반환
    #     """

    #     # 1) Request_Random_SequenceT 객체 생성
    #     req = Request_Random_SequenceT()
    #     req.id = req_id
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/control/random_sequence",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Random_SequenceT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Control Random Sequence failed: obj_payload is None")

    #     return result["obj_payload"]

    async def control_dock(self, robot_model: str, req_id: str, command: str) -> ResponseDockControlT:
        """
        [도킹 제어]
        - command: 도킹 명령 ( "dock", "undock", "dockstop" )
        - ResponseDockT 객체 반환
        """

        # 1) RequestDockControlT 객체 생성
        req = RequestDockControlT()
        req.id = req_id
        req.command = command

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/dock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDockControlT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Undock failed: obj_payload is None")

        return result["obj_payload"]

    async def control_charge_trigger(self, robot_model: str, req_id: str, control: bool) -> ResponseChargeTriggerT:
        """
        [충전 트리거 제어]
        - control: 충전 트리거 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseChargeTriggerT 객체 반환
        """
        # 1) RequestChargeTriggerT 객체 생성
        req = RequestChargeTriggerT()
        req.id = req_id
        req.control = control

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/chargeTrigger",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseChargeTriggerT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Charge Trigger failed: obj_payload is None")

        return result["obj_payload"]

    async def control_get_obs_box(self, robot_model: str, req_id: str) -> ResponseGetObsBoxT:
        """
        [장애물 박스 조회]
        - ResponseGetObsBoxT 객체 반환
        """

        # 1) RequestGetObsBoxT 객체 생성
        req = RequestGetObsBoxT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/getObsBox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetObsBoxT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Obs Box failed: obj_payload is None")

        return result["obj_payload"]

    async def control_set_obs_box(self, robot_model: str, req_id: str, min_x: float, min_y: float, min_z: float, max_x: float, max_y: float, max_z: float, map_range: float) -> ResponseGetObsBoxT:
        """
        [장애물 박스 설정]
        - min_x: 최소 X 좌표
        - min_y: 최소 Y 좌표
        - min_z: 최소 Z 좌표
        - max_x: 최대 X 좌표
        - max_y: 최대 Y 좌표
        - max_z: 최대 Z 좌표
        - map_range: 맵 범위
        - ResponseSetObsBox 객체 반환
        """

        # 1) RequestSetObsBoxT 객체 생성
        req = RequestSetObsBoxT()
        req.id = req_id
        minT = ObsBoxT()
        minT.x = min_x
        minT.y = min_y
        minT.z = min_z
        maxT = ObsBoxT()
        maxT.x = max_x
        maxT.y = max_y
        maxT.z = max_z
        req.min = minT
        req.max = maxT
        req.range = map_range

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/setObsBox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetObsBoxT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Obs Box failed: obj_payload is None")

        return result["obj_payload"]

    async def control_led_mode(self, robot_model: str, req_id: str, control: bool, color: str) -> ResponseLedModeT:
        """
        [LED 제어]
        - req_id: 요청 ID
        - control: LED 조작 모드 (True: 수동조작, False: 자동조작)
        - color: LED 색상
        - ResponseLedModeT 객체 반환
        """

        # 1) RequestLedModeT 객체 생성
        req = RequestLedModeT()
        req.id = req_id
        req.control = control
        req.color = color

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/led",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseLedModeT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control LED failed: obj_payload is None")

        return result["obj_payload"]

    async def control_motor_mode(self, robot_model: str, req_id: str, control: bool) -> ResponseMotorModeT:
        """
        [모터 제어]
        - req_id: 요청 ID
        - control: 모터 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseMotorModeT 객체 반환
        """

        # 1) RequestMotorModeT 객체 생성
        req = RequestMotorModeT()
        req.id = req_id
        req.control = control

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/motor",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMotorModeT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]

    async def control_jog_mode(self, robot_model: str, req_id: str, control: bool) -> ResponseJogT:
        """
        [조이스틱 모드 제어]
        - req_id: 요청 ID
        - control: 조이스틱 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseJogT 객체 반환
        """

        # 1) RequestJogT 객체 생성
        req = RequestJogT()
        req.id = req_id
        req.control = control

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/jog",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseJogT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]

    async def control_sensor_mode(self, robot_model: str, req_id: str, command: str, control: bool, frequency: int) -> ResponseSensorModeT:
        """
        [센서 모드 제어]
        - req_id: 요청 ID
        - command: 센서 종류 ( "camera", "lidar2d", "lidar3d" )
        - control: 센서 켜기/끄기 (True: 켜기, False: 끄기)
        - frequency: 센서 주파수 (Hz)
        - ResponseSensorModeT 객체 반환
        """

        # 1) RequestSensorModeT 객체 생성
        req = RequestSensorModeT()
        req.id = req_id
        req.command = command
        req.control = control
        req.frequency = frequency

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/sensor",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSensorModeT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]

    async def control_path_mode(self, robot_model: str, req_id: str, control: bool, frequency: int) -> ResponsePathModeT:
        """
        [경로 모드 제어]
        - req_id: 요청 ID
        - control: 경로 전송 켜기/끄기 (True: 켜기, False: 끄기)
        - frequency: 경로 전송 주파수 (Hz)
        - ResponsePathModeT 객체 반환
        """

        # 1) RequestPathModeT 객체 생성
        req = RequestPathModeT()
        req.id = req_id
        req.control = control
        req.frequency = frequency

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/path",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponsePathModeT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]

    async def control_detect_marker(self, robot_model: str, req_id: str, command: str, camera_number: int, camera_serial: str, marker_size: float) -> ResponseDetectMarkerT:
        """
        [마커 감지 제어]
        - req_id: 요청 ID
        - command: 감지 명령 ( "charuco", "aruco" )
        - camera_number: 카메라 번호
        - camera_serial: 카메라 시리얼 번호
        - marker_size: 마커 크기
        - ResponseDetectMarkerT 객체 반환
        """

        # 1) RequestDetectMarkerT 객체 생성
        req = RequestDetectMarkerT()
        req.id = req_id
        req.command = command
        req.number = camera_number
        req.serial = camera_serial
        req.m_size = marker_size

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/detectMarker",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDetectMarkerT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]
