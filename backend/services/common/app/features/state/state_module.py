import asyncio
import time

import psutil
import rb_database.mongo_db as mongo_db
from app.features.info.info_module import InfoService
from app.features.program.program_module import ProgramService
from app.socket.socket_client import socket_client
from fastapi import HTTPException
from flat_buffers.IPC.Request_CallWhoAmI import Request_CallWhoAmIT
from flat_buffers.IPC.Request_PowerControl import (
    Request_PowerControlT,
)
from flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from flat_buffers.IPC.Response_CallWhoamI import Response_CallWhoamIT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from flat_buffers.IPC.State_Core import State_CoreT
from rb_modules.log import rb_log
from rb_resources.file import read_json_file
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError
from utils.asyncio_helper import fire_and_log
from utils.helper import get_current_ip
from utils.parser import t_to_dict

from .state_schema import StateMessageItemPD

program_service = ProgramService()
info_service = InfoService()
zenoh_client = ZenohClient()


class StateService:
    def __init__(self):
        self._all_connect_state_period = 1
        self._prev_all_connected_status = None
        self._robot_models = read_json_file("data", "robot_models.json")

    async def get_system_state(self, namespaces: list[str]):
        ip = await get_current_ip()

        all_connected = "STABLE"
        core_sw_list = []

        for namespace in namespaces:
            robot_model = self._robot_models.get(namespace)

            if not robot_model:
                print(f"no robot model: {namespace}", flush=True)
                continue

            core_sw_list.append(
                {
                    "sw_name": namespace,
                    "alias": robot_model["alias"],
                    "be_service": robot_model["be_service"],
                    "model": robot_model["model"],
                    "connected": "DISCONNECT",
                }
            )

        for core_sw in core_sw_list:
            try:
                if core_sw["be_service"] == "manipulate":
                    topic, mv, obj, attachment = await zenoh_client.receive_one(
                        f"{core_sw["sw_name"]}/state_core", flatbuffer_obj_t=State_CoreT, timeout=1
                    )

                    if obj["statusOutColl"] == 1:
                        core_sw["connected"] = "OUT_COLLISION"
                    elif obj["statusSelfColl"] == 1:
                        core_sw["connected"] = "SELF_COLLISION"
                    elif obj["statusSwitchEmg"] == 1:
                        core_sw["connected"] = "EMERGENCY_STOP"
                    elif obj["statusPowerOut"] == 0:
                        core_sw["connected"] = "POWER_OFF"
                    elif obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 1:
                        core_sw["connected"] = "POWER_CHECKED"
                    elif obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 2:
                        core_sw["connected"] = "COMM_CHECKED"
                    elif obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 3:
                        core_sw["connected"] = "PARAM_CHECKED"
                    elif obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 4:
                        core_sw["connected"] = "JOINT_CHECKED"
                    elif obj["statusPowerOut"] == 1 or obj["statusServoNum"] == 5:
                        core_sw["connected"] = "SYSTEM_CHECKED"
                    elif obj["statusPowerOut"] == 1 and obj["statusServoNum"] == 6:
                        core_sw["connected"] = "STABLE"

                elif core_sw["be_service"] == "mobility":
                    topic, mv, obj, attachment = await zenoh_client.receive_one(
                        f"{core_sw.namespace}/state",
                    )
                elif core_sw["be_service"] == "sensor":
                    topic, mv, obj, attachment = await zenoh_client.receive_one(
                        f"{core_sw.namespace}/state_core",
                    )
            except (
                asyncio.exceptions.TimeoutError,
                ZenohNoReply,
                ZenohReplyError,
                ZenohTransportError,
            ):
                core_sw["connected"] = "DISCONNECT"

        statuses = [sw["connected"] for sw in core_sw_list]
        if all(s == "STABLE" for s in statuses):
            all_connected = "STABLE"
        elif all(s == "DISCONNECT" for s in statuses):
            all_connected = "DISCONNECT"
        else:
            all_connected = "UNSTABLE"

        return {
            "ip": ip,
            "all_connected": all_connected,
            "cpu_usage": psutil.cpu_percent(interval=None),
            "core_sw_list": core_sw_list,
        }

    async def repeat_get_system_state(self):
        try:
            await mongo_db.wait_db_ready()

            next_ts = time.monotonic()

            col = mongo_db.db["robot_info"]

            while True:
                doc = await col.find_one({}, {"_id": 0, "components": 1}) or {}

                raw_components = doc.get("components")
                namespaces = list(raw_components or [])

                if not namespaces:
                    continue

                res = await self.get_system_state(namespaces=namespaces)

                fire_and_log(socket_client.emit("system_state", res))

                next_ts += self._all_connect_state_period
                await asyncio.sleep(max(0, next_ts - time.monotonic()))
        except Exception as e:
            rb_log.error(f"repeat_get_system_state {e}")

    def get_all_robots_info(self):
        try:
            req = Request_CallWhoAmIT()

            results = zenoh_client.query_all(
                "*/call_whoami",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_CallWhoamIT,
                flatbuffer_buf_size=1024,
                timeout=5,
            )

            return results
        except (
            asyncio.exceptions.TimeoutError,
            ZenohNoReply,
            ZenohReplyError,
            ZenohTransportError,
        ):
            return list([])

    async def get_state_message(self, *, topic: str, message: StateMessageItemPD):
        rb_log.debug("🔎 subscribe */state_message")

        sw_name = topic.split("/")[0]

        message["swName"] = sw_name
        await socket_client.emit("state_message", message)

    async def power_control(
        self, *, power_option: int, sync_servo: bool, stoptime: int | None = 0.5
    ):
        try:
            rb_log.debug(f"power_control {power_option} {sync_servo} {stoptime}")
            req = Request_PowerControlT()
            req.power_option = power_option

            if sync_servo and power_option == 0:
                res = await self.servo_control(servo_option=power_option)

            if power_option == 0 and stoptime is not None:
                await program_service.call_smoothjog_stop(stoptime=stoptime)

            res = zenoh_client.query_one(
                "C500920/call_powercontrol",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=32,
            )

            if sync_servo and power_option == 1:
                res = await self.servo_control(servo_option=power_option)

            return t_to_dict(res)

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: {e}")) from e

    async def servo_control(self, *, servo_option: int):
        try:
            req = Request_ServoControlT()
            req.servoOption = servo_option

            res = zenoh_client.query_one(
                "*/call_servocontrol",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=32,
            )

            return res["dict_payload"]

        except (ZenohNoReply, ZenohReplyError, ZenohTransportError):
            raise
        except Exception as e:
            raise HTTPException(status_code=502, detail=str(f"error: servo {e}")) from e
