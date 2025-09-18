import flatbuffers
from app.modules.program.program_module_service import ProgramService
from fastapi import HTTPException
from flat_buffers.IPC.Request_PowerControl import (
    Request_PowerControlT,
)
from flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh import zenoh_client
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.parser import t_to_dict

programService = ProgramService()


class StateService:
    def __init__(self):
        pass

    async def get_all_robots_info(self):
        try:
            results = await zenoh_client.query("whoami", timeout=2000)

            services = []
            for r in results:
                try:
                    info = r.payload.decode("utf-8")
                except Exception:
                    info = str(r.payload)
                services.append(
                    {
                        "peer": str(r.key_expr),
                        "info": info,
                    }
                )

            return services
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

    async def power_control(
        self, *, power_option: int, sync_servo: bool, stoptime: int | None = 0.5
    ):
        try:
            req = Request_PowerControlT()
            req.power_option = power_option

            res = zenoh_client.query_one(
                "*/call_powercontrol", flatbuffer_obj=req, flatbuffer_buf_size=32
            )

            if sync_servo:
                res = await self.servo_control(servo_option=power_option)

                if power_option == 0 and stoptime is not None:
                    await programService.call_smoothjog_stop(stoptime=stoptime)

            return t_to_dict(res)

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

    async def servo_control(self, *, servo_option: int):
        try:
            req = Request_ServoControlT()
            req.servo_option = servo_option

            b = flatbuffers.Builder(32)
            b.Finish(req.Pack(b))
            fb_payload = bytes(b.Output())

            res = zenoh_client.query_one("*/call_servocontrol", payload=fb_payload)

            buf = res["payload"]
            res = Response_FunctionsT.InitFromPackedBuf(buf, 0)

            return t_to_dict(res)

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e
