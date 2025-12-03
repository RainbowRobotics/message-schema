from app.features.program.program_module import ProgramService
from rb_flat_buffers.IPC.Request_PowerControl import Request_PowerControlT
from rb_flat_buffers.IPC.Request_ReferenceControl import Request_ReferenceControlT
from rb_flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_zenoh.client import ZenohClient

from .state_schema import (
    ReferenceControlRequestPD,
    ServoControlRequestPD,
)

program_service = ProgramService()
zenoh_client = ZenohClient()


class StateService(BaseService):
    def __init__(self):
        pass

    async def call_powercontrol(
        self, *, robot_model: str, power_option: int, sync_servo: bool, stoptime: int | None = 3
    ):
        # 1. Power OFF 시퀀스 (Reference OFF -> SmoothJog Stop -> Servo ON)
        if power_option == 0:
            dict_reference_res = await self.call_referencecontrol(
                robot_model=robot_model, request=ReferenceControlRequestPD(reference_option=0)
            )
            if dict_reference_res.get("returnValue") != 0:
                dict_reference_res["target"] = "call_referencecontrol"
                return dict_reference_res

            if stoptime is not None:
                dict_smoothjog_stop_res = await program_service.call_smoothjog_stop(
                    robot_model=robot_model, stoptime=stoptime
                )
                if dict_smoothjog_stop_res.get("returnValue") != 0:
                    dict_smoothjog_stop_res["target"] = "call_smoothjog_stop"
                    return dict_smoothjog_stop_res

            # 1-3. Servo Control ON (Sync Servo 옵션)
            if sync_servo:
                dict_servo_res = await self.call_servocontrol(
                    robot_model=robot_model, request=ServoControlRequestPD(servo_option=1)
                )
                if dict_servo_res.get("returnValue") != 0:
                    dict_servo_res["target"] = "call_servocontrol"
                    return dict_servo_res

        # 2. Power Control (Main Zenoh Call)
        req = Request_PowerControlT()
        req.powerOption = power_option

        power_res = zenoh_client.query_one(
            f"{robot_model}/call_powercontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        
        dict_power_res = power_res["dict_payload"]

        # 3. Power ON 시퀀스 (Servo Control ON)
        if power_option == 1 and sync_servo:
            dict_servo_res = await self.call_servocontrol(
                robot_model=robot_model, request=ServoControlRequestPD(servo_option=1)
                )
            if dict_servo_res.get("returnValue") != 0:
                dict_servo_res["target"] = "call_servocontrol"
                return dict_servo_res

        dict_power_res["target"] = "call_powercontrol"
        return dict_power_res


    async def call_servocontrol(self, *, robot_model: str, request : ServoControlRequestPD):
        req = Request_ServoControlT()
        req.servoOption = request.servo_option

        servo_res = zenoh_client.query_one(
            f"{robot_model}/call_servocontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return servo_res["dict_payload"]


    async def call_referencecontrol(self, *, robot_model: str, request: ReferenceControlRequestPD):
        req = Request_ReferenceControlT()
        req.refcontrolOption = request.reference_option

        reference_res = zenoh_client.query_one(
            f"{robot_model}/call_referencecontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return reference_res["dict_payload"]
