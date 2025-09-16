import flatbuffers
from fastapi import HTTPException
from flat_buffers.IPC.Request_PowerControl import (
    Request_PowerControlAddPowerOption,
    Request_PowerControlEnd,
    Request_PowerControlStart,
)
from flat_buffers.IPC.Request_ReferenceControl import (
    Request_ReferenceControlAddRefcontrolOption,
    Request_ReferenceControlEnd,
    Request_ReferenceControlStart,
)
from flat_buffers.IPC.Request_ServoControl import (
    Request_ServoControlAddServoOption,
    Request_ServoControlEnd,
    Request_ServoControlStart,
)
from flat_buffers.IPC.Response_Functions import Response_Functions
from rb_zenoh import zenoh_client
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError


class StateService:
    def __init__(self):
        pass

    async def power_control(self, *, robot_model: str, power_option: int, sync_servo: bool):
        # ObjectAPI
        # try:
        #     req = Request_PowerControlT()
        #     req.powerOption = 1

        #     b = flatbuffers.Builder(128)
        #     b.Finish(req.Pack(b))
        #     fb_bytes = bytes(b.Output())

        #     res = zenoh_client.query_one("example/function_powercontrol", payload=fb_bytes)

        #     buf = res["payload"]
        #     res = Response_PowerControlT.InitFromPackedBuf(buf, 0)

        # except Exception as e:
        #     raise HTTPException(status_code=502, detail=str(f"flatbuffers decode error: {e}")) from e

        # return res

        # TableAPI
        try:
            b = flatbuffers.Builder(32)
            Request_PowerControlStart(b)
            Request_PowerControlAddPowerOption(b, power_option)
            root = Request_PowerControlEnd(b)
            b.Finish(root)
            power_fb_payload = bytes(b.Output())

            power_res = zenoh_client.query_one(
                f"{robot_model}/call_powercontrol", payload=power_fb_payload
            )

            tbl = Response_Functions.GetRootAs(power_res["payload"], 0)
            power_return_value = tbl.ReturnValue()

            if power_return_value != 0:
                return {
                    "error": True,
                    "error_target": "call_powercontrol",
                    "return_value": power_return_value,
                }

            return_value = power_return_value

            if power_option == 0:
                self.reference_control(robot_model=robot_model, reference_option=0)
            elif power_option == 1 and sync_servo:
                servo_return_value = self.servo_control(robot_model=robot_model, servo_option=1)

                return_value = servo_return_value["return_value"]

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

        return {"return_value": return_value}

    async def servo_control(self, *, robot_model: str, servo_option: int):
        try:
            b = flatbuffers.Builder(32)
            Request_ServoControlStart(b)
            Request_ServoControlAddServoOption(b, servo_option)
            root = Request_ServoControlEnd(b)
            b.Finish(root)
            servo_fb_payload = bytes(b.Output())

            servo_res = zenoh_client.query_one(
                f"{robot_model}/call_servocontrol", payload=servo_fb_payload
            )

            tbl = Response_Functions.GetRootAs(servo_res["payload"], 0)
            return_value = tbl.ReturnValue()

            if return_value != 0:
                return {
                    "error": True,
                    "error_target": "call_servocontrol",
                    "return_value": return_value,
                }

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(
                status_code=502, detail=str(f"flatbuffers decode error: {e}")
            ) from e

        return {"return_value": return_value}

    async def reference_control(self, *, robot_model: str, reference_option: int):
        try:
            b = flatbuffers.Builder(32)
            Request_ReferenceControlStart(b)
            Request_ReferenceControlAddRefcontrolOption(b, reference_option)
            root = Request_ReferenceControlEnd(b)
            b.Finish(root)
            reference_fb_payload = bytes(b.Output())

            reference_res = zenoh_client.query_one(
                f"{robot_model}/call_referencecontrol", payload=reference_fb_payload
            )

            tbl = Response_Functions.GetRootAs(reference_res["payload"], 0)
            return_value = tbl.ReturnValue()

            if return_value != 0:
                return {
                    "error": True,
                    "error_target": "call_referencecontrol",
                    "return_value": return_value,
                }

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(
                status_code=502, detail=str(f"flatbuffers decode error: {e}")
            ) from e

        return {"return_value": return_value}
