"""State Module"""
from rb_flat_buffers.IPC.Request_PowerControl import Request_PowerControlT
from rb_modules.service import BaseService
from rb_sdk.manipulate_sdk.manipulate_move import RBManipulateMoveSDK
from rb_sdk.manipulate_sdk.manipulate_state import RBManipulateStateSDK
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from .state_schema import (
    Request_ReferenceControlPD,
    Request_ServoControlPD,
)

zenoh_client = ZenohClient()

manipulate_state_sdk = RBManipulateStateSDK()
manipulate_move_sdk = RBManipulateMoveSDK()

class StateService(BaseService):
    """State Service"""

    def call_powercontrol(
        self, *, robot_model: str, power_option: int, sync_servo: bool, stoptime: int | None = 3
    ):
        """
        [Power Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            power_option: 파워 옵션
            sync_servo: 서보 동기화 여부
            stoptime: 정지 시간
        """
        # 1. Power OFF 시퀀스 (Reference OFF -> SmoothJog Stop -> Servo ON)
        if power_option == 0:
            dict_reference_res = manipulate_state_sdk.call_referencecontrol(
                robot_model=robot_model, reference_option=0
            )
            if dict_reference_res.returnValue != 0:
                return {
                    "target": "call_referencecontrol",
                    "returnValue": dict_reference_res.returnValue,
                }

            if stoptime is not None:
                dict_smoothjog_stop_res = manipulate_move_sdk.call_smoothjog_stop(
                    robot_model=robot_model, stop_time=stoptime
                )
                if dict_smoothjog_stop_res.returnValue != 0:
                    return {
                        "target": "call_smoothjog_stop",
                        "returnValue": dict_smoothjog_stop_res.returnValue,
                    }

            if sync_servo:
                dict_servo_res = manipulate_state_sdk.call_servocontrol(
                    robot_model=robot_model, servo_option=1
                )
                if dict_servo_res.returnValue != 0:
                    return {
                        "target": "call_servocontrol",
                        "returnValue": dict_servo_res.returnValue,
                    }

        # 2. Power Control (Main Zenoh Call)
        req = Request_PowerControlT()
        req.powerOption = power_option

        power_res = manipulate_state_sdk.call_powercontrol(
            robot_model=robot_model, power_option=power_option
        )

        if power_res.returnValue != 0:
            return {
                "target": "call_powercontrol",
                "returnValue": power_res.returnValue,
            }

        # 3. Power ON 시퀀스 (Servo Control ON)
        if power_option == 1 and sync_servo:
            dict_servo_res = manipulate_state_sdk.call_servocontrol(
                robot_model=robot_model, servo_option=1
            )

            if dict_servo_res.returnValue != 0:
                return {
                    "target": "call_servocontrol",
                    "returnValue": dict_servo_res.returnValue,
                }

        return {
            "target": "call_powercontrol",
            "returnValue": power_res.returnValue,
        }


    def call_servocontrol(self, *, robot_model: str, request : Request_ServoControlPD):
        """
        [Servo Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 서보 옵션
        """
        servo_res = manipulate_state_sdk.call_servocontrol(
            robot_model=robot_model, servo_option=request.servo_option
        )

        return t_to_dict(servo_res)


    def call_referencecontrol(self, *, robot_model: str, request: Request_ReferenceControlPD):
        """
        [Reference Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            request: 참조 옵션
        """
        reference_res = manipulate_state_sdk.call_referencecontrol(
            robot_model=robot_model, reference_option=request.reference_option
        )

        return t_to_dict(reference_res)
