from app.modules.program.program_module_service import ProgramService
from flat_buffers.IPC.Request_PowerControl import Request_PowerControlT
from flat_buffers.IPC.Request_ReferenceControl import Request_ReferenceControlT
from flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh import zenoh_client

program_service = ProgramService()


class StateService:
    def __init__(self):
        pass

    async def power_control(
        self, *, robot_model: str, power_option: int, sync_servo: bool, stoptime: int | None = 3
    ):
        if power_option == 0:
            dict_reference_res = await self.reference_control(
                robot_model=robot_model, reference_option=0
            )

            reference_return_value = dict_reference_res.get(
                "returnValue", dict_reference_res.get("return_value", None)
            )

            if reference_return_value != 0:
                dict_reference_res["target"] = "call_referencecontrol"
                return dict_reference_res

            if stoptime is not None:
                dict_smoothjog_stop_res = await program_service.call_smoothjog_stop(
                    robot_model=robot_model, stoptime=stoptime
                )

                smoothjog_stop_return_value = dict_smoothjog_stop_res.get(
                    "returnValue", dict_smoothjog_stop_res.get("return_value", None)
                )

                if smoothjog_stop_return_value != 0:
                    dict_smoothjog_stop_res["target"] = "call_smoothjog_stop"
                    return dict_smoothjog_stop_res

        elif power_option == 1 and sync_servo:
            dict_servo_res = await self.servo_control(robot_model=robot_model, servo_option=1)

            servo_return_value = dict_servo_res.get(
                "returnValue", dict_servo_res.get("return_value", None)
            )

            if servo_return_value != 0:
                dict_servo_res["target"] = "call_servocontrol"
                return dict_servo_res

        req = Request_PowerControlT()
        req.powerOption = power_option

        power_res = zenoh_client.query_one(
            f"{robot_model}/call_powercontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        dict_power_res = power_res["dict_payload"]

        dict_power_res["target"] = "call_powercontrol"

        return dict_power_res

    async def servo_control(self, *, robot_model: str, servo_option: int):
        req = Request_ServoControlT()
        req.servoOption = servo_option

        servo_res = zenoh_client.query_one(
            f"{robot_model}/call_servocontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return servo_res["dict_payload"]

    async def reference_control(self, *, robot_model: str, reference_option: int):
        req = Request_ReferenceControlT()
        req.refcontrolOption = reference_option

        reference_res = zenoh_client.query_one(
            f"{robot_model}/call_referencecontrol",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        return reference_res["dict_payload"]
