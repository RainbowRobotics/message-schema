

from rb_flat_buffers.SLAMNAV.Request_Dock import Request_DockT
from rb_flat_buffers.SLAMNAV.Response_Dock import Response_DockT
from rb_flat_buffers.SLAMNAV.Request_Charge_Trigger import Request_Charge_TriggerT
from rb_flat_buffers.SLAMNAV.Response_Charge_Trigger import Response_Charge_TriggerT
from rb_flat_buffers.SLAMNAV.State_Change_Dock import State_Change_DockT
from rb_flat_buffers.SLAMNAV.Request_Get_Obs_Box import Request_Get_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Response_Get_Obs_Box import Response_Get_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Request_Set_Obs_Box import Request_Set_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Response_Set_Obs_Box import Response_Set_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Request_Jog import Request_JogT
from rb_flat_buffers.SLAMNAV.Response_Jog import Response_JogT
from rb_flat_buffers.SLAMNAV.Request_Sensor import Request_SensorT
from rb_flat_buffers.SLAMNAV.Response_Sensor import Response_SensorT
from rb_flat_buffers.SLAMNAV.Request_Path import Request_PathT
from rb_flat_buffers.SLAMNAV.Response_Path import Response_PathT
from rb_flat_buffers.SLAMNAV.Request_Detect import Request_DetectT
from rb_flat_buffers.SLAMNAV.Response_Detect import Response_DetectT
from rb_flat_buffers.SLAMNAV.Request_Led import Request_LedT
from rb_flat_buffers.SLAMNAV.Request_Motor import Request_MotorT
from rb_flat_buffers.SLAMNAV.Response_Led import Response_LedT
from rb_flat_buffers.SLAMNAV.Response_Motor import Response_MotorT
from rb_flat_buffers.SLAMNAV.Request_Get_Safety_Field import Request_Get_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Request_Set_Safety_Field import Request_Set_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Response_Get_Safety_Field import Response_Get_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Response_Set_Safety_Field import Response_Set_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Request_Get_Safety_Flag import Request_Get_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Request_Set_Safety_Flag import Request_Set_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Response_Get_Safety_Flag import Response_Get_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Response_Set_Safety_Flag import Response_Set_Safety_FlagT

from ..base import RBBaseSDK
from .schema.amr_control_schema import SlamnavControlPort


class RBAmrControlSDK(RBBaseSDK,SlamnavControlPort):
    """Rainbow Robotics AMR Control SDK"""

    async def control_frequency(self, robot_model: str, req_id: str, target: str, onoff: bool, frequency: int) -> Response_FrequencyT:
        """
        Control Frequency 전송]

        Args:
            robot_model: 로봇 모델명
            req_id: 요청 ID
            target: 대상
            onoff: 켜기/끄기
            frequency: 주기

        Returns:
            Response_FrequencyT: 응답 객체
        """

        # # 1) Request_FrequencyT 객체 생성
        # req = Request_FrequencyT()
        # req.id = req_id
        # req.target = target
        # req.onoff = onoff
        # req.frequency = frequency

        # # 2) 요청 전송
        # result = self.zenoh_client.query_one(
        #     f"{robot_model}/control/frequency",
        #     flatbuffer_req_obj=req,
        #     flatbuffer_res_T_class=Response_FrequencyT,
        #     flatbuffer_buf_size=125,
        # )

        # # 3) 결과 처리 및 반환
        # if result["obj_payload"] is None:
        #     raise RuntimeError("Call Control Frequency failed: obj_payload is None")

        # return result["obj_payload"]

    async def control_led(self, robot_model: str, req_id: str, onoff: bool, color: str) -> Response_LedT:
        """
        [Control LED 전송]
        - model: ControlRequestModel
        - Response_Control_LEDT 객체 반환
        """

        # 1) Request_Control_LEDT 객체 생성
        req = Request_LedT()
        req.id = req_id
        req.onoff = onoff
        req.color = color

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/led",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_LedT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control LED failed: obj_payload is None")

        return result["obj_payload"]

    async def control_motor(self, robot_model: str, req_id: str, onoff: bool) -> Response_MotorT:
        """
        [Control Motor 전송]
        - model: ControlRequestModel
        - Response_Control_MotorT 객체 반환
        """
        # 1) Request_Control_MotorT 객체 생성
        req = Request_MotorT()
        req.id = req_id
        req.onoff = onoff

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/motor",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_MotorT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        return result["obj_payload"]

    async def control_dock(self, robot_model: str, req_id: str) -> Response_DockT:
        """
        [Control Dock 전송]
        - model: ControlRequestModel
        - Response_DockT 객체 반환
        """
        # 1) Request_DockT 객체 생성
        req = Request_DockT()
        req.id = req_id
        req.command = "dock"

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/dock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_DockT,
            flatbuffer_buf_size=125,
        )
        print("============== control_dock ===============")
        print(result)

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Undock failed: obj_payload is None")

        return result["obj_payload"]

    async def control_undock(self, robot_model: str, req_id: str) -> Response_UndockT:
        """
        [Control Undock 전송]
        - model: ControlRequestModel
        - Response_UndockT 객체 반환
        """
        # 1) Request_UndockT 객체 생성
        req = Request_UndockT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/dock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_UndockT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Undock failed: obj_payload is None")

        return result["obj_payload"]

    async def control_dock_stop(self, robot_model: str, req_id: str) -> Response_DockStopT:
        """
        [Control Dock Stop 전송]
        - model: ControlRequestModel
        - Response_DockStopT 객체 반환
        """
        # 1) Request_DockStopT 객체 생성
        req = Request_DockStopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/dockStop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_DockStopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Dock Stop failed: obj_payload is None")

        return result["obj_payload"]

    async def control_obs_box(self, robot_model: str, req_id: str, command: str, min_x: float, min_y: float, min_z: float, max_x: float, max_y: float, max_z: float, map_range: float) -> Response_Obs_BoxT:
        """
        [Control Obs Box 전송]
        - model: ControlRequestModel
        - Response_Obs_BoxT 객체 반환
        """
        # 1) Request_Obs_BoxT 객체 생성
        req = Request_Obs_BoxT()
        req.id = req_id
        req.command = command
        req.minX = min_x
        req.minY = min_y
        req.minZ = min_z
        req.maxX = max_x
        req.maxY = max_y
        req.maxZ = max_z
        req.mapRange = map_range

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/obs_box",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Obs_BoxT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Obs Box failed: obj_payload is None")

        return result["obj_payload"]

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
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/safety_field",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Safety_FieldT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety Field failed: obj_payload is None")

        return result["obj_payload"]

    async def control_reset_safety_flag(self, robot_model: str, req_id: str, reset_flag: list[Safety_FlagT]) -> Response_Reset_Safety_FlagT:
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
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/reset_safety_flag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Reset_Safety_FlagT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Reset Safety Flag failed: obj_payload is None")

        return result["obj_payload"]

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
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/safety_io",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Safety_IoT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety IO failed: obj_payload is None")

        return result["obj_payload"]

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
        result = self.zenoh_client.query_one(
            f"{robot_model}/control/random_sequence",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Random_SequenceT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Random Sequence failed: obj_payload is None")

        return result["obj_payload"]
