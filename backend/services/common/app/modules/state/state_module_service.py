import asyncio
import socket
import time

import flatbuffers
import psutil
from app.modules.program.program_module_service import ProgramService
from app.socket import socket_client
from fastapi import HTTPException
from flat_buffers.IPC.Request_CallWhoAmI import Request_CallWhoAmIT
from flat_buffers.IPC.Request_PowerControl import (
    Request_PowerControlT,
)
from flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from flat_buffers.IPC.Response_CallWhoamI import Response_CallWhoamIT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from flat_buffers.IPC.State_Core import State_CoreT
from rb_zenoh import zenoh_client
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.parser import t_to_dict

programService = ProgramService()


class StateService:
    def __init__(self):
        self._all_connect_state_period = 1
        self._prev_all_connected_status = None

    async def get_system_state(self):
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        res = self.get_all_robots_info()

        all_connected = "DISCONNECT"
        core_sw_list = []

        for r in res:
            if r["err"] is not None:
                continue

            core_sw_info = r["dict_payload"]
            core_sw_info["connected"] = "DISCONNECT"

            be_service = r["dict_payload"]["category"]
            robot_namespace = r["dict_payload"]["name"]

            if be_service == "manipulate":
                topic, mv, obj, attachment = await zenoh_client.receive_one(
                    f"{robot_namespace}/state_core",
                    flatbuffer_obj_t=State_CoreT,
                )

                if obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 1:
                    all_connected = "CONNECTED"
                    core_sw_info["connected"] = "CONNECTED"
                elif obj["statusPowerOut"] == 1 or obj["statusServoNum"] == 1:
                    all_connected = "UNSTABLE"
                    core_sw_info["connected"] = "UNSTABLE"

            elif be_service == "mobility":
                topic, mv, obj, attachment = await zenoh_client.receive_one(
                    f"{robot_namespace}/state",
                )
            elif be_service == "sensor":
                topic, mv, obj, attachment = await zenoh_client.receive_one(
                    f"{robot_namespace}/state_core",
                )

            core_sw_list.append(core_sw_info)

        return {
            "ip": ip,
            "all_connected": all_connected,
            "cpu_usage": psutil.cpu_percent(interval=1),
            "core_sw_num": len(core_sw_list),
            "core_sw_list": core_sw_list,
        }

    async def repeat_get_system_state(self):
        next_ts = time.monotonic()

        while True:
            res = await self.get_system_state()

            if self._prev_all_connected_status != res and socket_client.connected:
                await socket_client.emit("system_state", res)
                self._prev_all_connected_status = res

            next_ts += self._all_connect_state_period
            await asyncio.sleep(max(0, next_ts - time.monotonic()))

    def get_all_robots_info(self):
        req = Request_CallWhoAmIT()

        results = zenoh_client.query_all(
            "*/call_whoami",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_CallWhoamIT,
            flatbuffer_buf_size=1024,
        )

        return results

    async def power_control(
        self, *, power_option: int, sync_servo: bool, stoptime: int | None = 0.5
    ):
        try:
            req = Request_PowerControlT()
            req.power_option = power_option

            res = zenoh_client.query_one(
                "*/call_powercontrol",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=32,
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
