import asyncio
import time

from app.socket.socket_client import socket_client
from fastapi import HTTPException
from flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from flat_buffers.IPC.State_Core import State_CoreT
from rb_database import mongo_db
from rb_modules.log import rb_log
from rb_resources.json import read_json_file
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.asyncio_helper import fire_and_log

zenoh_client = ZenohClient()


class ProgramService:
    def __init__(self) -> None:
        self._robot_models = read_json_file("robot_models.json")

    async def get_all_speedbar(self, *, components: list[str]):
        try:
            diff_flag = False
            min_speedbar = 1.0

            for component in components:
                model_info = self._robot_models.get(component)
                be_service = model_info.get("be_service")
                if be_service == "manipulate":
                    topic, mv, obj, attachment = await zenoh_client.receive_one(
                        f"{component}/state_core", flatbuffer_obj_t=State_CoreT, timeout=0.5
                    )

                    speedbar = obj["motionSpeedBar"] or 1.0

                    if speedbar < min_speedbar:
                        min_speedbar = speedbar
                        diff_flag = True

                elif be_service == "mobility":
                    # TODO: mobility speedbar
                    continue
                elif be_service == "sensor":
                    # TODO: sensor speedbar
                    continue
                else:
                    # TODO: other speedbar
                    continue

            if diff_flag:
                await self.control_speed_bar(components=components, speedbar=min_speedbar)

            return {"speedbar": min_speedbar}
        except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
            rb_log.error(f"get_all_speedbar zenoh error: {e}", disable_db=True)
        except Exception as e:
            rb_log.error(f"get_all_speedbar {e}")

    async def repeat_get_all_speedbar(self):
        try:
            await mongo_db.wait_db_ready()
            next_ts = time.monotonic()
            col = mongo_db.db["robot_info"]

            while True:
                if col is None:
                    await asyncio.sleep(0.5)
                    continue

                doc = await col.find_one({}, {"_id": 0, "components": 1}) or {}
                components = list(doc.get("components") or [])

                res = await self.get_all_speedbar(components=components)
                if isinstance(res, dict) and "speedbar" in res:
                    fire_and_log(socket_client.emit("speedbar", res))
                else:
                    rb_log.error(f"repeat_get_all_speedbar res: {res}")

                next_ts += 1
                await asyncio.sleep(max(0, next_ts - time.monotonic()))

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
            rb_log.error(f"repeat_get_all_speedbar zenoh error: {e}", disable_db=True)
        except Exception as e:
            rb_log.error(f"repeat_get_all_speedbar {e}")

    async def control_speed_bar(self, *, components: list[str], speedbar: int):
        try:
            failed_component = []
            rb_log.debug(f"components>{components}")
            for component in components:
                model_info = self._robot_models.get(component)
                rb_log.debug(f"model_info => ${model_info}")
                be_service = model_info.get("be_service")
                if be_service == "manipulate":
                    req = Request_MotionSpeedBarT()
                    req.alpha = speedbar

                    res = zenoh_client.query_one(
                        f"{component}/call_speedbar",
                        flatbuffer_req_obj=req,
                        flatbuffer_buf_size=32,
                        flatbuffer_res_T_class=Response_FunctionsT,
                    )

                    if res["err"]:
                        failed_component.append(component)
                        rb_log.error(res["err"])
                elif be_service == "mobility":
                    # TODO: mobility speedbar
                    pass
                elif be_service == "sensor":
                    # TODO: sensor speedbar
                    pass
                else:
                    # TODO: other speedbar
                    pass

            return {"returnValue": 500 if len(failed_component) > 0 else 0}

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
            rb_log.error(f"control_speed_bar zenoh error: {e}", disable_db=True)
            raise
        except Exception as e:
            rb_log.error(f"control_speed_bar error: {e}", disable_db=True)
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

    async def call_smoothjog_stop(self, *, stoptime: float):
        try:
            req = Request_Move_SmoothJogStopT()
            req.stoptime = stoptime

            res = zenoh_client.query_one(
                "*/call_smoothjog_stop",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=32,
            )

            return res["dict_payload"]
        except (ZenohNoReply, ZenohReplyError, ZenohTransportError) as e:
            rb_log.error(f"call_smoothjog_stop zenoh error: {e}", disable_db=True)
            raise
        except Exception as e:
            rb_log.error(f"call_smoothjog_stop error: {e}", disable_db=True)
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e
