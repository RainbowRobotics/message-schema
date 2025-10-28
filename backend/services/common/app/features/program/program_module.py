from collections.abc import Iterable
from datetime import UTC, datetime
from typing import Any

from bson import ObjectId
from fastapi import HTTPException
from pymongo import ReturnDocument
from pymongo.operations import UpdateOne
from rb_database import MongoDB
from rb_database.utils import make_check_include_query, make_check_search_text_query
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flow_manager.control import RB_Flow_Manager_ProgramState
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError

from .program_schema import (
    Request_Create_FlowPD,
    Request_Create_Multiple_FlowPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Delete_FlowsPD,
    Request_Delete_TasksPD,
    Request_Update_FlowPD,
    Request_Update_Multiple_FlowPD,
    Request_Update_ProgramPD,
)

zenoh_client = ZenohClient()


class ProgramService(BaseService):
    def __init__(self) -> None:
        pass

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

    def build_task_tree(self, tasks: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
        """
        평면 tasks를 트리로 변환한다.
        """

        nodes: dict[str, dict] = {}
        for t in tasks:
            nodes[str(t["taskId"])] = t.copy()

        children_map: dict[str, list[dict]] = {}
        roots: list[dict] = []

        for n in nodes.values():
            pid = n.get("nodeId")
            if pid and pid in nodes:
                children_map.setdefault(pid, []).append(n)
            else:
                roots.append(n)

        def sort_children(lst: list[dict]):
            lst.sort(key=lambda x: x.get("order", 0))

        VISITING, VISITED = 1, 2
        state: dict[str, int] = {}

        def attach(node: dict):
            nid = node["taskId"]
            if state.get(nid) == VISITING:
                return
            if state.get(nid) == VISITED:
                return
            state[nid] = VISITING

            kids = children_map.get(nid)
            if kids:
                sort_children(kids)
                node["steps"] = kids
                for c in kids:
                    attach(c)

            state[nid] = VISITED

        sort_children(roots)
        for r in roots:
            attach(r)

        return roots

    async def get_task(self, *, task_id: str, db: MongoDB):
        tasks_col = db["tasks"]
        task_doc = await tasks_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=404, detail="Task not found")

        task_doc["taskId"] = str(task_doc.pop("_id"))

        return task_doc

    async def get_task_list(self, *, flow_id: str, db: MongoDB):
        tasks_col = db["tasks"]

        query: dict[str, Any] = {}

        query = make_check_include_query("flowId", flow_id, query=query)

        tasks_docs = await tasks_col.find(query).sort("order", 1).to_list(length=None)

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))

        return {
            "tasks": tasks_docs,
            "taskTree": self.build_task_tree(tasks_docs),
        }

    async def upsert_tasks(self, *, request: Request_Create_Multiple_TaskPD, db: MongoDB):
        now = datetime.now(UTC).isoformat()

        tasks_col = db["tasks"]
        flows_col = db["flows"]

        tasks = [t_to_dict(task) for task in request.tasks]

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

    async def delete_tasks(self, *, request: Request_Delete_TasksPD, db: MongoDB):
        request_dict = t_to_dict(request)
        task_ids = [ObjectId(tid) for tid in request_dict["task_ids"]]

        tasks_col = db["tasks"]
        task_deleted_res = await tasks_col.delete_many(
            {"$or": [{"_id": {"$in": task_ids}}, {"nodeId": {"$in": request_dict["task_ids"]}}]}
        )

        return {
            "taskDeleted": task_deleted_res.deleted_count,
        }

    async def create_flows(self, *, request: Request_Create_Multiple_FlowPD, db: MongoDB):
        flows_col = db["flows"]
        program_col = db["programs"]

        flows = [t_to_dict(flow) for flow in request.flows]

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

        flows = [t_to_dict(flow) for flow in request.flows]

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

    async def delete_flows(self, *, request: Request_Delete_FlowsPD, db: MongoDB):
        request_dict = t_to_dict(request)
        flow_ids = request_dict["flow_ids"]

        flows_col = db["flows"]
        tasks_col = db["tasks"]

        flow_deleted_res = await flows_col.delete_many(
            {"_id": {"$in": [ObjectId(fid) for fid in flow_ids]}}
        )
        task_deleted_res = await tasks_col.delete_many({"flowId": {"$in": flow_ids}})

        return {
            "flowDeleted": flow_deleted_res.deleted_count,
            "taskDeleted": task_deleted_res.deleted_count,
        }

    async def get_program_info(self, *, program_id: str, db: MongoDB):
        program_col = db["programs"]
        flows_col = db["flows"]

        program_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not program_doc:
            raise HTTPException(status_code=400, detail="Program not found")

        program_doc["programId"] = str(program_doc.pop("_id"))

        flows_docs = await flows_col.find({"programId": program_id}).to_list(length=None)

        for doc in flows_docs:
            doc["flowId"] = str(doc.pop("_id"))

        return {
            "program": program_doc,
            "flows": flows_docs,
        }

    async def get_program_list(
        self,
        *,
        state: RB_Flow_Manager_ProgramState | None = None,
        search_name: str | None = None,
        db: MongoDB,
    ):
        program_col = db["programs"]

        query: dict[str, Any] = {}

        if search_name:
            if len(search_name) > 100:
                raise ValueError("search_name must be less than 100 characters")

            query = make_check_search_text_query("name", search_name, query=query)

        query = make_check_include_query("state", state, query=query)

        program_docs = await program_col.find(query).to_list(length=None)

        for doc in program_docs:
            doc["programId"] = str(doc.pop("_id"))

        return program_docs

    async def create_program_and_flows(self, *, request: Request_Create_ProgramPD, db: MongoDB):
        try:
            now = datetime.now(UTC)

            program_doc = {
                **t_to_dict(request.program),
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

    async def update_program_and_flows(self, *, request: Request_Update_ProgramPD, db: MongoDB):
        try:
            now = datetime.now(UTC)

            program_doc = {
                **t_to_dict(request.program),
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
