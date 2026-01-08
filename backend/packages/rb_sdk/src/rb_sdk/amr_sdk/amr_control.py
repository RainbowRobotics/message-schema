
from rb_flat_buffers.SLAMNAV.Request_Control_Frequency import Request_Control_FrequencyT
from rb_flat_buffers.SLAMNAV.Request_Control_LED import Request_Control_LEDT
from rb_flat_buffers.SLAMNAV.Request_Control_Motor import Request_Control_MotorT
from rb_flat_buffers.SLAMNAV.Request_Dock import Request_DockT
from rb_flat_buffers.SLAMNAV.Request_Obs_Box import Request_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Request_Random_Sequence import Request_Random_SequenceT
from rb_flat_buffers.SLAMNAV.Request_Reset_Safety_Flag import Request_Reset_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Request_Safety_Field import Request_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Request_Safety_Io import Request_Safety_IoT
from rb_flat_buffers.SLAMNAV.Response_Control_Frequency import Response_Control_FrequencyT
from rb_flat_buffers.SLAMNAV.Response_Control_LED import Response_Control_LEDT
from rb_flat_buffers.SLAMNAV.Response_Control_Motor import Response_Control_MotorT
from rb_flat_buffers.SLAMNAV.Response_Dock import Response_DockT
from rb_flat_buffers.SLAMNAV.Response_Obs_Box import Response_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Response_Random_Sequence import Response_Random_SequenceT
from rb_flat_buffers.SLAMNAV.Response_Reset_Safety_Flag import Response_Reset_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Response_Safety_Field import Response_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Response_Safety_Io import Response_Safety_IoT
from rb_zenoh.client import ZenohClient

from .schema.amr_control_schema import SlamnavControlPort


class RBAmrControlSDK(SlamnavControlPort):
    """Rainbow Robotics AMR Control SDK"""
    client: ZenohClient
    def __init__(self, client: ZenohClient):
        self.client = client

    async def control_frequency(self, robot_model: str, req_id: str, target: str, onoff: bool, frequency: int) -> Response_Control_FrequencyT:
        """
        [Control Frequency 전송]
        - model: ControlRequestModel
        - Response_Control_FrequencyT 객체 반환
        """
        # 1) Request_Control_FrequencyT 객체 생성
        req = Request_Control_FrequencyT()
        req.id = req_id
        req.target = target
        req.onoff = onoff
        req.frequency = frequency

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/frequency",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Control_FrequencyT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_led(self, robot_model: str, req_id: str, onoff: bool, color: str) -> Response_Control_LEDT:
        """
        [Control LED 전송]
        - model: ControlRequestModel
        - Response_Control_LEDT 객체 반환
        """
        # 1) Request_Control_LEDT 객체 생성
        req = Request_Control_LEDT()
        req.id = req_id
        req.onoff = onoff
        req.color = color

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/led",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Control_LEDT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_motor(self, robot_model: str, req_id: str, onoff: bool) -> Response_Control_MotorT:
        """
        [Control Motor 전송]
        - model: ControlRequestModel
        - Response_Control_MotorT 객체 반환
        """
        # 1) Request_Control_MotorT 객체 생성
        req = Request_Control_MotorT()
        req.id = req_id
        req.onoff = onoff

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/motor",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Control_MotorT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_dock(self, robot_model: str, req_id: str, command: str) -> Response_DockT:
        """
        [Control Dock 전송]
        - model: ControlRequestModel
        - Response_DockT 객체 반환
        """
        # 1) Request_DockT 객체 생성
        req = Request_DockT()
        req.id = req_id
        req.command = command

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/dock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_DockT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_obs_box(self, robot_model: str, req_id: str, command: str, obs_box: int) -> Response_Obs_BoxT:
        """
        [Control Obs Box 전송]
        - model: ControlRequestModel
        - Response_Obs_BoxT 객체 반환
        """
        # 1) Request_Obs_BoxT 객체 생성
        req = Request_Obs_BoxT()
        req.id = req_id
        req.command = command
        req.obs_box = obs_box

        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/obs_box",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Obs_BoxT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_safety_field(self, robot_model: str, req_id: str, command: str, safety_field: int) -> Response_Safety_FieldT:
        """
        [Control Safety Field 전송]
        - model: ControlRequestModel
        - Response_Safety_FieldT 객체 반환
        """
        # 1) Request_Safety_FieldT 객체 생성
        req = Request_Safety_FieldT()
        req.id = req_id
        req.command = command
        req.safety_field = safety_field
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/safety_field",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Safety_FieldT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_reset_safety_flag(self, robot_model: str, req_id: str, reset_flag: str) -> Response_Reset_Safety_FlagT:
        """
        [Control Reset Safety Flag 전송]
        - model: ControlRequestModel
        - Response_Reset_Safety_FlagT 객체 반환
        """
        # 1) Request_Reset_Safety_FlagT 객체 생성
        req = Request_Reset_Safety_FlagT()
        req.id = req_id
        req.reset_flag = reset_flag
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/reset_safety_flag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Reset_Safety_FlagT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_safety_io(self, robot_model: str, req_id: str, command: str, mcu0_dio: list[bool], mcu1_dio: list[bool]) -> Response_Safety_IoT:
        """
        [Control Safety IO 전송]
        - model: ControlRequestModel
        - Response_Safety_IoT 객체 반환
        """
        # 1) Request_Safety_IoT 객체 생성
        req = Request_Safety_IoT()
        req.id = req_id
        req.command = command
        req.mcu0_dio = mcu0_dio
        req.mcu1_dio = mcu1_dio
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/safety_io",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Safety_IoT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def control_random_sequence(self, robot_model: str, req_id: str) -> Response_Random_SequenceT:
        """
        [Control Random Sequence 전송]
        - model: ControlRequestModel
        - Response_Random_SequenceT 객체 반환
        """
        # 1) Request_Random_SequenceT 객체 생성
        req = Request_Random_SequenceT()
        req.id = req_id
        # 2) 요청 전송
        result = self.client.query_one(
            f"{robot_model}/control/random_sequence",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Random_SequenceT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]
