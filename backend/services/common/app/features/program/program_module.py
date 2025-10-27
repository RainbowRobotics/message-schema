import asyncio
import time
from datetime import UTC, datetime
from typing import Any

from bson import ObjectId
from fastapi import HTTPException
from pymongo import ReturnDocument
from pymongo.operations import UpdateOne
from rb_database import MongoDB, mongo_db
from rb_flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_resources.file import read_json_file
from rb_utils.asyncio_helper import fire_and_log
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError

from app.socket.socket_client import socket_client

from .program_schema import (
    Request_Create_FlowPD,
    Request_Create_Multiple_FlowPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Update_FlowPD,
    Request_Update_Multiple_FlowPD,
    Request_Update_ProgramPD,
)

zenoh_client = ZenohClient()


class ProgramService(BaseService):
    def __init__(self) -> None:
        self._robot_models = read_json_file("data", "robot_models.json")

    async def get_all_speedbar(self, *, components: list[str]):
        try:
            # diff_flag = False
            min_speedbar = 1.0

            for component in components:
                model_info = self._robot_models.get(component)
                be_service = model_info.get("be_service")
                if be_service == "manipulate":
                    topic, mv, obj, attachment = await zenoh_client.receive_one(
                        f"{component}/state_core", flatbuffer_obj_t=State_CoreT, timeout=0.5
                    )

                    speedbar = 1.0

                    if obj:
                        speedbar = obj["motionSpeedBar"]

                    if speedbar < min_speedbar:
                        min_speedbar = speedbar
                        # diff_flag = True

                elif be_service == "mobility":
                    # TODO: mobility speedbar
                    continue
                elif be_service == "sensor":
                    # TODO: sensor speedbar
                    continue
                else:
                    # TODO: other speedbar
                    continue

            # if diff_flag:
            #     await self.control_speed_bar(components=components, speedbar=min_speedbar)

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
                    next_ts = time.monotonic() + 1.0
                    continue

                doc = await col.find_one({}, {"_id": 0, "components": 1}) or {}
                components = list(doc.get("components") or [])

                res = await self.get_all_speedbar(components=components)
                if isinstance(res, dict) and "speedbar" in res:
                    fire_and_log(socket_client.emit("speedbar", res))
                else:
                    rb_log.error(f"repeat_get_all_speedbar res: {res}")

                now = time.monotonic()
                next_ts = max(next_ts + 1.0, now + 1.0)
                await asyncio.sleep(max(0.0, next_ts - now))

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

    # def upload_program_python_script(self, *, script_name: str, script_content: str):
    #     try:
    #         col = mongo_db.db["program_script"]
    #         await col.insert_one({"script_name": script_name, "script_content": script_content})
    #     except Exception as e:
    #         raise e

    # async def get_task_status(self, *, robot_model: str, task_id: str):
    #     col = mongo_db.db["tasks"]

    #     doc = await col.find_one(
    #         {"robot_model": robot_model, "_id": task_id},
    #     )

    #     if not doc:
    #         return None

    #     doc = dict(doc)

    #     doc["task_id"] = doc.pop("_id")

    #     return doc

    # async def set_task_status(self, *, task_id: str, request: Request_Set_Task_StatusPD):
    #     status = request["status"]
    #     program_id = request["program_id"]
    #     sync_task_ids = request["sync_task_ids"]
    #     node_path = request["node_path"]
    #     offset = request["offset"]
    #     updated_at = datetime.now(UTC)

    #     col = mongo_db.db["tasks"]

    #     doc = await col.update_one(
    #         {"_id": task_id},
    #         {
    #             "$set": {
    #                 "status": status,
    #                 "program_id": program_id,
    #                 "sync_task_ids": sync_task_ids,
    #                 "node_path": node_path,
    #                 "offset": offset,
    #                 "updated_at": updated_at,
    #             }
    #         },
    #         upsert=True,
    #     )

    #     if doc.get("status") == ProgramStatus.ERROR:
    #         rb_log.error(f"Task error: {task_id}")

    #     req = RB_Program_Task_StatusT()

    #     req.status = status
    #     req.taskId = task_id
    #     req.programId = program_id
    #     req.syncTaskIds = sync_task_ids
    #     req.nodePath = node_path
    #     req.offset = offset

    #     zenoh_client.publish("/change-task-status", flatbuffer_req_obj=req, flatbuffer_buf_size=100)

    #     return doc

    # async def set_task_checkpoint(self, *, task_id: str, request: Request_Set_Task_CheckpointPD):
    #     program_id = request["program_id"]
    #     sync_task_ids = request["sync_task_ids"]
    #     node_path = request["node_path"]
    #     offset = request["offset"]
    #     updated_at = datetime.now(UTC)

    #     col = mongo_db.db["tasks"]

    #     doc = await col.update_one(
    #         {"_id": task_id},
    #         {
    #             "$set": {

    async def upsert_tasks(self, *, request: Request_Create_Multiple_TaskPD, db: MongoDB):
        now = datetime.now(UTC).isoformat()

        tasks_col = db["tasks"]
        flows_col = db["flows"]

        tasks = [task.model_dump() for task in request.tasks]

        flow_ids = {t["flowId"] for t in tasks}
        record_find_flow_ids = []

        for fid in flow_ids:
            if fid not in record_find_flow_ids:
                continue

            find_doc = await flows_col.find_one({"_id": ObjectId(fid)})

            if not find_doc:
                raise HTTPException(status_code=400, detail=f"Invalid flowId: {fid}")

            record_find_flow_ids.append(fid)

        ops: list[UpdateOne] = []
        index_map: list[tuple[int, dict[str, Any]]] = []  # (tasks 인덱스, 기존 taskId)

        for i, doc in enumerate(tasks):
            doc["updatedAt"] = now
            set_on_insert = {}

            tid = doc.pop("taskId", None)

            q = {"_id": ObjectId(tid)}

            if tid is not None:
                find_task_doc = await tasks_col.find_one(q, {"_id": 0, "createdAt": 1})

                if find_task_doc:
                    doc["createdAt"] = find_task_doc["createdAt"]
            else:
                set_on_insert = {"createdAt": now}

            ops.append(
                UpdateOne(
                    q,
                    {"$set": doc, "$setOnInsert": set_on_insert},
                    upsert=True,
                )
            )

            index_map.append(
                (
                    i,
                    {
                        "taskId": tid,
                        "createdAt": now,
                    },
                )
            )

        res = await tasks_col.bulk_write(ops, ordered=False)

        print(f"res: {res}", flush=True)

        for op_idx, oid in (res.upserted_ids or {}).items():
            i, o_doc = index_map[op_idx]
            tasks[i]["taskId"] = str(oid)
            tasks[i]["createdAt"] = o_doc["createdAt"]

        return tasks

    async def create_flows(self, *, request: Request_Create_Multiple_FlowPD, db: MongoDB):
        flows_col = db["flows"]
        program_col = db["programs"]

        flows = [flow.model_dump() for flow in request.flows]

        program_ids = {f["programId"] for f in flows}
        record_find_program_ids = []

        for pid in program_ids:
            if pid not in record_find_program_ids:
                continue

            if not await program_col.find_one({"_id": ObjectId(pid)}):
                raise HTTPException(status_code=400, detail=f"Invalid programId: {pid}")

            record_find_program_ids.append(pid)

        res = await flows_col.insert_many(flows)

        for flow, inserted_id in zip(flows, res.inserted_ids, strict=False):
            flow["flowId"] = str(inserted_id)
            flow.pop("_id", None)

        return flows

    async def update_flows(self, *, request: Request_Update_Multiple_FlowPD, db: MongoDB):
        now = datetime.now(UTC).isoformat()
        ops = []

        flows_col = db["flows"]
        program_col = db["programs"]

        flows = [flow.model_dump(exclude_none=True) for flow in request.flows]

        program_ids = {f["programId"] for f in flows}

        record_find_program_ids = []
        updated_flow_ids = []

        for pid in program_ids:
            if pid not in record_find_program_ids:
                continue

            if not await program_col.find_one({"_id": ObjectId(pid)}):
                raise HTTPException(status_code=400, detail=f"Invalid programId: {pid}")

            record_find_program_ids.append(pid)

        for flow in flows:
            fid = flow.pop("flowId")
            flow["updatedAt"] = now
            ops.append(UpdateOne({"_id": ObjectId(fid)}, {"$set": flow}))
            updated_flow_ids.append(fid)

        if ops:
            await flows_col.bulk_write(ops, ordered=False)

        res = await flows_col.find(
            {"_id": {"$in": [ObjectId(fid) for fid in updated_flow_ids]}}
        ).to_list(length=None)

        for doc in res:
            doc["flowId"] = str(doc.pop("_id"))

        return {
            "flows": res,
        }

    async def update_program_and_flows(self, *, request: Request_Update_ProgramPD, db: MongoDB):
        try:
            now = datetime.now(UTC)

            program_doc = {
                **request.program.model_dump(exclude_none=True),
                "updatedAt": now.isoformat(),
            }

            col = db["programs"]

            find_doc = await col.find_one({"_id": ObjectId(program_doc["programId"])})

            if not find_doc:
                raise HTTPException(status_code=400, detail="Program not found")

            program_res = await col.find_one_and_update(
                {"_id": ObjectId(program_doc["programId"])},
                {"$set": program_doc},
                return_document=ReturnDocument.AFTER,
            )

            program_res["programId"] = str(program_res.pop("_id"))

            for flow in request.flows:
                flow.programId = str(program_res["programId"])
                flow.name = f'{flow.robotModel}_{program_res["name"]}'
                flow.scriptName = flow.name

            flows_res = await self.update_flows(
                request=Request_Update_Multiple_FlowPD(
                    flows=[
                        Request_Update_FlowPD(**f.model_dump(exclude_none=True))
                        for f in request.flows
                    ]
                ),
                db=db,
            )

            return {
                "program": program_res,
                "flows": flows_res["flows"],
            }
        except Exception as e:
            raise e

    async def create_program_and_flows(self, *, request: Request_Create_ProgramPD, db: MongoDB):
        try:
            now = datetime.now(UTC)

            program_doc = {
                **request.program.model_dump(exclude_none=True),
                "createdAt": now.isoformat(),
                "updatedAt": now.isoformat(),
            }

            col = db["programs"]

            find_doc = await col.find_one({"name": program_doc["name"]})

            if find_doc:
                raise HTTPException(status_code=400, detail="Program already exists")

            program_res = await col.insert_one(program_doc)

            find_program_doc = await col.find_one({"_id": program_res.inserted_id})

            if find_program_doc:
                find_program_doc["programId"] = str(program_res.inserted_id)
                find_program_doc.pop("_id", None)

            for flow in request.flows:
                flow.programId = str(program_res.inserted_id)
                flow.name = f'{flow.robotModel}_{program_doc["name"]}'
                flow.scriptName = flow.name
                flow.createdAt = now.isoformat()
                flow.updatedAt = now.isoformat()

            flows_res = await self.create_flows(
                request=Request_Create_Multiple_FlowPD(
                    flows=[
                        Request_Create_FlowPD(**f.model_dump(exclude_none=True))
                        for f in request.flows
                    ]
                ),
                db=db,
            )

        except Exception as e:
            raise e

        return {
            "program": find_program_doc,
            "flows": flows_res,
        }

    async def delete_program(self, *, program_id: str, db: MongoDB):
        program_col = db["programs"]
        flows_col = db["flows"]
        tasks_col = db["tasks"]

        program_deleted_res = await program_col.delete_one({"_id": ObjectId(program_id)})

        find_flow_res = await flows_col.find({"programId": program_id}, {"_id": 1}).to_list(
            length=None
        )

        find_flow_ids = [str(flow["_id"]) for flow in find_flow_res]

        flow_deleted_res = await flows_col.delete_many({"programId": program_id})

        task_deleted_count = 0

        for fid in find_flow_ids:
            task_deleted_res = await tasks_col.delete_many({"flowId": fid})

            task_deleted_count += task_deleted_res.deleted_count

        return {
            "programDeleted": program_deleted_res.deleted_count,
            "flowDeleted": flow_deleted_res.deleted_count,
            "taskDeleted": task_deleted_count,
        }
