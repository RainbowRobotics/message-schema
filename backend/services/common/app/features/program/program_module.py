import asyncio
import codecs
import contextlib
import importlib.util
import os
import sys
from collections.abc import Iterable
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, cast

from bson import ObjectId
from fastapi import HTTPException
from motor.motor_asyncio import AsyncIOMotorClientSession
from pymongo import ReadPreference, ReturnDocument, WriteConcern
from pymongo.errors import PyMongoError
from pymongo.operations import UpdateOne
from rb_database import MongoDB, get_db
from rb_database.utils import make_check_include_query, make_check_search_text_query
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller
from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.schema import RB_Flow_Manager_ProgramState
from rb_flow_manager.step import Step
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_utils.asyncio_helper import fire_and_log
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError

from app.socket.socket_client import socket_client

from .program_schema import (
    Request_Create_Multiple_StepPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Create_TaskPD,
    Request_Delete_StepsPD,
    Request_Delete_TasksPD,
    Request_Get_Script_ContextPD,
    Request_Preview_Start_ProgramPD,
    Request_Preview_Stop_ProgramPD,
    Request_Program_ExecutionPD,
    Request_Tasks_ExecutionPD,
    Request_Update_Multiple_TaskPD,
    Request_Update_ProgramPD,
    Request_Update_StepStatePD,
    Request_Update_TaskPD,
)

zenoh_client = ZenohClient()


class ProgramService(BaseService):
    def __init__(self) -> None:
        zenoh_controller = Zenoh_Controller()
        self.script_executor = ScriptExecutor(controller=zenoh_controller)
        self._script_base_path = Path("/app/data/common/scripts")

    async def call_resume_or_pause(
        self,
        *,
        db: MongoDB,
        is_pause: bool,
        # program_id: str | None = None,
        # flow_id: str | None = None,
    ):
        """
        Resume 또는 Pause를 호출하는 함수.
        """

        # state = (
        #     RB_Flow_Manager_ProgramState.PAUSED
        #     if is_pause
        #     else RB_Flow_Manager_ProgramState.RUNNING
        # )
        req_manipulate_resume_or_pause = Request_MotionPauseT()

        res_manipulate_resume_or_pause: dict[str, Any] | None = None

        if is_pause:
            res_manipulate_resume_or_pause = zenoh_client.query_one(
                "*/call_pause",
                flatbuffer_req_obj=req_manipulate_resume_or_pause,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=2,
            )
        else:
            res_manipulate_resume_or_pause = zenoh_client.query_one(
                "*/call_resume",
                flatbuffer_req_obj=req_manipulate_resume_or_pause,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=2,
            )

        # if program_id:
        #     await self.update_program_state(program_id=program_id, state=state, db=db)
        # if flow_id:
        #     await self.update_flow_state(flow_id=flow_id, state=state, db=db)

        return {
            "manipulateReturnValue": (
                res_manipulate_resume_or_pause["dict_payload"]["returnValue"]
                if res_manipulate_resume_or_pause
                else None
            ),
        }

    async def call_smoothjog_stop(self, *, stoptime: float):
        """
        SmoothJog Stop을 호출하는 함수.
        """

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

    def build_step_tree(self, steps: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
        """
        평면 steps를 트리로 변환한다.
        """

        nodes: dict[str, dict] = {}
        for step in steps:
            stepId = step.get("stepId") or step.get("_id")
            step["stepId"] = str(stepId)
            step.pop("_id", None)
            nodes[step["stepId"]] = step.copy()

        children_map: dict[str, list[dict]] = {}
        roots: list[dict] = []

        for n in nodes.values():
            pid = n.get("parentStepId")
            if pid and pid in nodes:
                children_map.setdefault(pid, []).append(n)
            else:
                roots.append(n)

        def sort_children(lst: list[dict]):
            lst.sort(key=lambda x: x.get("order", 0))

        visiting, visited = 1, 2
        state: dict[str, int] = {}

        def attach(node: dict):
            nid = str(node.get("stepId"))
            if state.get(nid) == visiting:
                return
            if state.get(nid) == visited:
                return
            state[nid] = visiting

            kids = children_map.get(nid)
            if kids:
                sort_children(kids)
                node["children"] = kids
                for c in kids:
                    attach(c)

            state[nid] = visited

        sort_children(roots)
        for r in roots:
            attach(r)

        return roots

    async def get_step(self, *, step_id: str, db: MongoDB):
        """
        Step을 조회하는 함수.
        """

        steps_col = db["steps"]
        step_doc = await steps_col.find_one({"_id": ObjectId(step_id)})

        if not step_doc:
            raise HTTPException(status_code=404, detail="Step not found")

        step_doc["stepId"] = str(step_doc.pop("_id"))

        return step_doc

    async def get_step_list(self, *, task_id: str, db: MongoDB):
        """
        Steps들을 조회하는 함수.
        """

        steps_col = db["steps"]

        query: dict[str, Any] = {}

        query = make_check_include_query("taskId", task_id, query=query)

        steps_docs = await steps_col.find(query).sort("order", 1).to_list(length=None)

        for doc in steps_docs:
            doc["stepId"] = str(doc.pop("_id"))

        return {
            "steps": steps_docs,
            "stepTree": self.build_step_tree(steps_docs),
        }

    async def upsert_steps(self, *, request: Request_Create_Multiple_StepPD, db: MongoDB):
        """
        Steps들을 생성하거나 업데이트하는 함수.
        """

        request_dict = (
            {**request.model_dump(exclude_none=True, exclude_unset=True)}
            if hasattr(request, "model_dump")
            else t_to_dict(request)
        )

        now = datetime.now(UTC).isoformat()

        steps_col = db["steps"]
        tasks_col = db["tasks"]

        steps = [t_to_dict(step) for step in request_dict["steps"]]

        record_find_task_ids = []

        for step in steps:
            tid = step.get("taskId", None)
            pid = step.get("programId", None)

            if tid is None or tid in record_find_task_ids:
                continue

            if tid is None and step.get("stepId", None) is None:
                raise HTTPException(
                    status_code=400, detail="insert step without taskId is not allowed"
                )
            elif pid is None and step.get("stepId", None) is not None:
                raise HTTPException(
                    status_code=400, detail="insert step without programId is not allowed"
                )

            find_doc = await tasks_col.find_one({"_id": ObjectId(tid)})

            if not find_doc:
                raise HTTPException(status_code=400, detail=f"Invalid taskId: {tid}")

            record_find_task_ids.append(tid)

        ops: list[UpdateOne] = []
        index_map: list[tuple[int, dict[str, Any]]] = []  # (tasks 인덱스, 기존 taskId)

        for i, origin_doc in enumerate(steps):
            doc = request_dict["steps"][i] if origin_doc.get("stepId") is not None else origin_doc

            doc["updatedAt"] = now
            set_on_insert = {}

            sid = doc.pop("stepId", None)

            q = {"_id": ObjectId(sid)}

            if sid is not None:
                find_step_doc = await steps_col.find_one(q)

                if find_step_doc:
                    doc["createdAt"] = find_step_doc["createdAt"]
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
                        "stepId": sid,
                        "createdAt": now,
                    },
                )
            )

        res = await steps_col.bulk_write(ops, ordered=False)

        for op_idx, oid in (res.upserted_ids or {}).items():
            i, o_doc = index_map[op_idx]
            steps[i]["stepId"] = str(oid)
            steps[i]["createdAt"] = o_doc["createdAt"]

        for step in steps:
            tid = step.get("taskId", None)
            if tid is None:
                continue

            fire_and_log(self.write_script_context(tid, db), name="write_script_context")

        print(f"res: {res}", flush=True)

        return {
            "inserted_count": res.inserted_count,
            "matched_count": res.matched_count,
            "upserted_count": res.upserted_count,
            "modified_count": res.modified_count,
            "deleted_count": res.deleted_count,
            "upserted_ids": res.upserted_ids,
        }

    async def update_all_task_step_state(
        self, *, task_id: str, state: RB_Flow_Manager_ProgramState, db: MongoDB
    ):
        steps_col = db["steps"]
        tasks_col = db["tasks"]

        query: dict[str, Any] = {"scriptName": task_id}

        if ObjectId.is_valid(task_id):
            query = {"_id": ObjectId(task_id)}

        task_doc = await tasks_col.find_one(query)

        if not task_doc:
            return {
                "updated_count": 0,
            }

        real_task_id = str(task_doc["_id"])

        if state == RB_Flow_Manager_ProgramState.IDLE:
            fire_and_log(
                socket_client.emit(
                    f"program/task/{real_task_id}/ended",
                    {
                        "taskId": real_task_id,
                        "state": state,
                    },
                ),
                name="emit_task_ended",
            )

        update_res = await steps_col.update_many(
            {
                "taskId": real_task_id,
                "state": {"$ne": RB_Flow_Manager_ProgramState.ERROR},
            },
            {"$set": {"state": state}},
        )

        return {
            "updated_count": update_res.modified_count,
        }

    async def update_step_state(self, *, request: Request_Update_StepStatePD, db: MongoDB):
        request_dict = t_to_dict(request)

        step_id = request_dict["stepId"]
        state = request_dict["state"]

        steps_col = db["steps"]

        if not ObjectId.is_valid(step_id):
            return

        find_step_doc = await steps_col.find_one_and_update(
            {"_id": ObjectId(step_id)},
            {"$set": {"state": state}},
            return_document=ReturnDocument.AFTER,
        )

        if find_step_doc and state == RB_Flow_Manager_ProgramState.ERROR:
            await self.stop_program(
                request=Request_Program_ExecutionPD(programId=find_step_doc["programId"]), db=db
            )

        fire_and_log(
            socket_client.emit(
                f"program/task/{str(find_step_doc['_id'])}/update_state",
                {
                    "stepId": step_id,
                    "state": state,
                },
            ),
            name="emit_step_update_state",
        )

        # visited = set()

        # async def update_children(parent_id: str):
        #     if parent_id in visited:
        #         return
        #     visited.add(parent_id)

        #     children = await steps_col.find({"parentStepId": parent_id}).to_list(length=None)

        #     for child in children:
        #         cid = str(child["_id"])

        #         await steps_col.update_one(
        #             {"_id": child["_id"]},
        #             {"$set": {"state": state}},
        #         )

        #         await update_children(cid)

        # if find_step_doc:
        #     root_id = str(find_step_doc["_id"])
        #     await update_children(root_id)

        #     find_step_doc["stepId"] = root_id

        return find_step_doc

    async def delete_steps(self, *, request: Request_Delete_StepsPD, db: MongoDB):
        """
        Steps들을 삭제하는 함수.
        """

        request_dict = t_to_dict(request)
        step_ids = [ObjectId(sid) for sid in request_dict["step_ids"]]

        steps_col = db["steps"]
        step_deleted_res = await steps_col.delete_many(
            {
                "$or": [
                    {"_id": {"$in": step_ids}},
                    {"parentStepId": {"$in": request_dict["step_ids"]}},
                ]
            }
        )

        return {
            "stepDeleted": step_deleted_res.deleted_count,
        }

    async def write_script_context(self, task_id: str, db: MongoDB):
        """
        Steps를 컨텍스트로 변환하고 파일로 저장하는 함수.
        """
        task_col = db["tasks"]
        steps_col = db["steps"]
        program_col = db["programs"]

        task_doc = await task_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=400, detail=f"Invalid taskId: {task_id}")

        program_doc = await program_col.find_one({"_id": ObjectId(task_doc["programId"])})

        if not program_doc:
            raise HTTPException(
                status_code=400, detail=f"Invalid programId: {task_doc['programId']}"
            )

        steps_docs = await steps_col.find({"taskId": task_id}).to_list(length=None)

        steps_tree = self.build_step_tree(steps_docs)

        script_context = self.build_script_context(
            steps_tree, task_id, task_doc["scriptName"], program_doc["repeatCnt"]
        )

        os.makedirs(task_doc["scriptPath"], exist_ok=True)

        with open(
            f"{task_doc['scriptPath']}/{task_doc['scriptName']}.{task_doc['extension']}",
            "w",
            encoding="utf-8",
        ) as f:
            return f.write(script_context.encode("utf-8").decode("unicode_escape"))

    def parse_step_context(self, step: dict[str, Any], depth: int = 0, *, to_string: bool = False):
        """Steps tree를 컨텍스트로 변환하는 함수."""

        step_obj = Step.from_dict(step)
        return step_obj.to_py_string(depth) if to_string else step_obj

    def build_tree_from_client(self, steps_tree: list[dict[str, Any]], task_id: str):
        """
        프론트에서 받은 JSON(dict)을 실제 Step / RepeatStep 트리 객체로 변환하는 함수.
        """

        return Step(
            step_id=task_id,
            name=task_id,
            children=[self.parse_step_context(step, depth=8) for step in steps_tree],
        )

    def build_script_context(
        self, steps_tree: list[dict[str, Any]], task_id: str, script_name: str, repeat_count: int
    ):
        """
        Steps tree를 컨텍스트로 변환하는 함수.
        """
        body_context = "\n".join(
            self.parse_step_context(step, depth=8, to_string=True) for step in steps_tree
        )

        if not body_context:
            body_context = ""

        header_context = "from rb_flow_manager.executor import ScriptExecutor\n"
        header_context += "from rb_flow_manager.step import RepeatStep, Step\n"
        header_context += (
            "from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller\n\n"
        )
        header_context += "zenoh_controller = Zenoh_Controller()\n\n"

        root_block = (
            "tree = Step(\n"
            f"    step_id='{task_id}',\n"
            f"    name='{script_name}',\n"
            "    children=[\n"
            f"{body_context}\n"
            "    ],\n"
            ")\n"
        )

        footer_context = 'if __name__ == "__main__":\n'
        footer_context += "    executor = ScriptExecutor(controller=zenoh_controller)\n\n"
        footer_context += (
            f"    executor.start('{script_name}', tree, repeat_count={repeat_count})\n\n"
        )

        return header_context + "\n" + root_block + "\n" + footer_context

    async def get_script_context(self, *, request: Request_Get_Script_ContextPD, db: MongoDB):
        """
        Task의 스크립트 컨텍스트를 조회하는 함수.
        """
        request_dict = t_to_dict(request)
        task_id = request_dict["taskId"]
        steps_tree = request_dict["steps"]

        task_col = db["tasks"]
        program_col = db["programs"]

        task_doc = await task_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=400, detail=f"Invalid taskId: {task_id}")

        program_doc = await program_col.find_one({"_id": ObjectId(task_doc["programId"])})

        if not program_doc:
            raise HTTPException(
                status_code=400, detail=f"Invalid programId: {task_doc['programId']}"
            )

        script_context = self.build_script_context(
            steps_tree, task_id, task_doc["scriptName"], program_doc["repeatCnt"]
        )

        return {
            "context": codecs.decode(script_context, "unicode_escape"),
        }

    async def create_tasks(self, *, request: Request_Create_Multiple_TaskPD, db: MongoDB):
        """
        Task들을 생성하는 함수.
        """

        try:
            tasks_col = db["tasks"]
            program_col = db["programs"]

            request_dict = t_to_dict(request)

            tasks = request_dict["tasks"]

            for task in tasks:
                if not task["scriptPath"]:
                    task["scriptPath"] = str((self._script_base_path).resolve())

            program_ids = {t["programId"] for t in tasks}
            record_find_program_ids = []

            for pid in program_ids:
                if pid not in record_find_program_ids:
                    continue

                if not await program_col.find_one({"_id": ObjectId(pid)}):
                    raise HTTPException(status_code=400, detail=f"Invalid programId: {pid}")

                record_find_program_ids.append(pid)

            existed = await tasks_col.find(
                {"scriptName": {"$in": [task.get("scriptName") for task in tasks]}},
                {"scriptName": 1},
            ).to_list(length=None)

            if existed:
                exist_names = sorted({d["scriptName"] for d in existed})
                raise HTTPException(
                    status_code=409, detail=f"Already exists scriptName: {exist_names}"
                )

            res = await tasks_col.insert_many(tasks)

            for task, inserted_id in zip(tasks, res.inserted_ids, strict=False):
                task["taskId"] = str(inserted_id)
                task.pop("_id", None)

                fire_and_log(
                    self.write_script_context(task["taskId"], db), name="write_script_context"
                )

            return tasks
        except Exception as e:
            await tasks_col.delete_many(
                {"_id": {"$in": [ObjectId(task["taskId"]) for task in tasks]}}
            )
            raise e

    async def update_tasks(self, *, request: Request_Update_Multiple_TaskPD, db: MongoDB):
        """
        Task들을 업데이트하는 함수.
        """

        now = datetime.now(UTC).isoformat()
        ops = []

        tasks_col = db["tasks"]
        program_col = db["programs"]

        tasks = [t_to_dict(task) for task in request.tasks]

        program_ids = {t["programId"] for t in tasks}

        record_find_program_ids = []
        updated_task_ids = []

        for pid in program_ids:
            if pid in record_find_program_ids:
                continue

            if not await program_col.find_one({"_id": ObjectId(pid)}):
                raise HTTPException(status_code=400, detail=f"Invalid programId: {pid}")

            record_find_program_ids.append(pid)

        for task in tasks:
            tid = task.pop("taskId")
            task["updatedAt"] = now
            ops.append(UpdateOne({"_id": ObjectId(tid)}, {"$set": task}))
            updated_task_ids.append(tid)

        if ops:
            await tasks_col.bulk_write(ops, ordered=False)

        res = await tasks_col.find(
            {"_id": {"$in": [ObjectId(tid) for tid in updated_task_ids]}}
        ).to_list(length=None)

        for doc in res:
            doc["taskId"] = str(doc.pop("_id"))

        return {
            "tasks": res,
        }

    async def delete_tasks(self, *, request: Request_Delete_TasksPD, db: MongoDB):
        """
        Task들을 삭제하는 함수.
        """

        request_dict = t_to_dict(request)
        task_ids = request_dict["task_ids"]

        tasks_col = db["tasks"]
        steps_col = db["steps"]

        task_deleted_res = await tasks_col.delete_many(
            {"_id": {"$in": [ObjectId(tid) for tid in task_ids]}}
        )
        steps_deleted_res = await steps_col.delete_many({"taskId": {"$in": task_ids}})

        return {
            "taskDeleted": task_deleted_res.deleted_count,
            "stepDeleted": steps_deleted_res.deleted_count,
        }

    async def get_program_info(self, *, program_id: str, db: MongoDB):
        """
        Program과 Task를 동시에 조회하는 함수.
        """

        program_col = db["programs"]
        tasks_col = db["tasks"]

        program_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not program_doc:
            raise HTTPException(status_code=400, detail="Program not found")

        program_doc["programId"] = str(program_doc.pop("_id"))

        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))

        return {
            "program": program_doc,
            "tasks": tasks_docs,
        }

    async def get_program_list(
        self,
        *,
        state: RB_Flow_Manager_ProgramState | None = None,
        search_name: str | None = None,
        db: MongoDB,
    ):
        """
        Program 목록을 조회하는 함수.
        """

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

    async def create_program_and_tasks(self, *, request: Request_Create_ProgramPD, db: MongoDB):
        """
        Program과 Task를 동시에 생성하는 함수.
        """

        try:
            now = datetime.now(UTC)

            program_doc = {
                **t_to_dict(request.program),
                "createdAt": now.isoformat(),
                "updatedAt": now.isoformat(),
            }

            program_col = db["programs"]

            find_doc = await program_col.find_one({"name": program_doc["name"]})

            if find_doc:
                raise HTTPException(status_code=400, detail="Program already exists")

            program_res = await program_col.insert_one(program_doc)

            find_program_doc = await program_col.find_one({"_id": program_res.inserted_id})

            if find_program_doc:
                find_program_doc["programId"] = str(program_res.inserted_id)
                find_program_doc.pop("_id", None)

            for task in request.tasks:
                task.programId = str(program_res.inserted_id)
                task.name = f'{task.robotModel}_{program_doc["name"]}'
                task.scriptName = task.name
                task.scriptPath = ""
                task.createdAt = now.isoformat()
                task.updatedAt = now.isoformat()

            tasks_res = await self.create_tasks(
                request=Request_Create_Multiple_TaskPD(
                    tasks=[
                        Request_Create_TaskPD(**t.model_dump(exclude_none=True))
                        for t in request.tasks
                    ]
                ),
                db=db,
            )

        except Exception as e:
            await program_col.delete_one({"_id": program_res.inserted_id})
            raise e

        return {
            "program": find_program_doc,
            "tasks": tasks_res,
        }

    async def update_program_and_tasks(self, *, request: Request_Update_ProgramPD, db: MongoDB):
        """
        Program과 Task를 동시에 업데이트하는 함수.
        """

        try:
            now = datetime.now(UTC)

            program_doc = (
                {
                    **request.program.model_dump(exclude_none=True, exclude_unset=True),
                    "updatedAt": now.isoformat(),
                }
                if hasattr(request.program, "model_dump")
                else {
                    **t_to_dict(request.program),
                    "updatedAt": now.isoformat(),
                }
            )

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

            for task in request.tasks:
                task.programId = str(program_res["programId"])
                task.name = f'{task.robotModel}_{program_res["name"]}'
                task.scriptName = task.name

            tasks_res = await self.update_tasks(
                request=Request_Update_Multiple_TaskPD(
                    tasks=[
                        Request_Update_TaskPD(**t.model_dump(exclude_none=True, exclude_unset=True))
                        for t in request.tasks
                    ]
                ),
                db=db,
            )

            return {
                "program": program_res,
                "tasks": tasks_res["tasks"],
            }
        except Exception as e:
            raise e

    async def delete_program(self, *, program_id: str, db: MongoDB):
        """
        Program을 삭제하는 함수.
        """

        program_col = db["programs"]
        tasks_col = db["tasks"]
        steps_col = db["steps"]

        program_deleted_res = await program_col.delete_one({"_id": ObjectId(program_id)})
        task_deleted_res = await tasks_col.delete_many({"programId": program_id})
        step_deleted_res = await steps_col.delete_many({"programId": program_id})

        return {
            "programDeleted": program_deleted_res.deleted_count,
            "taskDeleted": task_deleted_res.deleted_count,
            "stepDeleted": step_deleted_res.deleted_count,
        }

    def _state_priority_expr(self, field="$state"):
        """
        Steps collection의 state를 우선순위로 변환하는 함수.
        """
        return {
            "$switch": {
                "branches": [
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.IDLE]}, "then": 0},
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.COMPLETED]}, "then": 1},
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.RUNNING]}, "then": 2},
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.PAUSED]}, "then": 3},
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.STOPPED]}, "then": 4},
                    {"case": {"$eq": [field, RB_Flow_Manager_ProgramState.ERROR]}, "then": 5},
                ],
                "default": 0,
            }
        }

    def _state_max_priority_expr(self):
        """
        Steps collection의 state를 재계산하는 함수.
        """
        return {
            "$switch": {
                "branches": [
                    {
                        "case": {"$eq": ["$maxPrio", 0]},
                        "then": RB_Flow_Manager_ProgramState.IDLE,
                    },
                    {
                        "case": {"$eq": ["$maxPrio", 1]},
                        "then": RB_Flow_Manager_ProgramState.COMPLETED,
                    },
                    {
                        "case": {"$eq": ["$maxPrio", 2]},
                        "then": RB_Flow_Manager_ProgramState.RUNNING,
                    },
                    {
                        "case": {"$eq": ["$maxPrio", 3]},
                        "then": RB_Flow_Manager_ProgramState.PAUSED,
                    },
                    {
                        "case": {"$eq": ["$maxPrio", 4]},
                        "then": RB_Flow_Manager_ProgramState.STOPPED,
                    },
                    {
                        "case": {"$eq": ["$maxPrio", 5]},
                        "then": RB_Flow_Manager_ProgramState.ERROR,
                    },
                ],
                "default": RB_Flow_Manager_ProgramState.IDLE,
            }
        }

    async def recompute_task_and_program(
        self, *, session: AsyncIOMotorClientSession, program_id: str, task_id: str, db: MongoDB
    ):
        """
        Task와 Program의 state를 재계산하는 함수.
        """
        steps_col = db["steps"]
        tasks_col = db["tasks"]
        programs_col = db["programs"]

        cursor = steps_col.aggregate(
            [
                {"$match": {"programId": program_id, "taskId": task_id}},
                {
                    "$group": {
                        "_id": {"programId": "$programId", "taskId": "$taskId"},
                        "maxPrio": {"$max": self._state_priority_expr("$state")},
                    }
                },
                {
                    "$addFields": {
                        "state": self._state_max_priority_expr(),
                    }
                },
                {
                    "$project": {
                        "_id": 0,
                        "programId": "$_id.programId",
                        "taskId": "$_id.taskId",
                        "state": 1,
                    }
                },
            ],
            session=session,
        )

        task_row = await cursor.to_list(length=1)
        task_status = task_row[0]["state"] if task_row else "idle"

        await tasks_col.update_one(
            {"_id": task_id},
            {"$set": {"state": task_status, "programId": program_id}},
            session=session,
            upsert=False,
        )

        cursor2 = tasks_col.aggregate(
            [
                {"$match": {"programId": program_id}},
                {
                    "$group": {
                        "_id": "$programId",
                        "maxPrio": {"$max": self._state_priority_expr("$state")},
                    }
                },
                {
                    "$addFields": {
                        "state": self._state_max_priority_expr(),
                    }
                },
                {"$project": {"_id": 0, "programId": "$_id", "state": 1}},
            ],
            session=session,
        )

        program_row = await cursor2.to_list(length=1)
        program_state = (
            program_row[0]["state"] if program_row else RB_Flow_Manager_ProgramState.IDLE
        )

        await programs_col.update_one(
            {"_id": ObjectId(program_id)},
            {"$set": {"state": program_state}},
            session=session,
            upsert=False,
        )

    async def _handle_step_change(self, db: MongoDB, change: dict[str, Any]):
        """
        Change Stream 이벤트를 받아 해당 (programId, taskId)만 부분 갱신.
        """
        op = change["operationType"]
        full = change.get("fullDocument")
        before = change.get("fullDocumentBeforeChange")
        doc_id = change["documentKey"]["_id"]

        program_id = None
        task_id = None

        steps_col = db["steps"]

        if op in ("insert", "replace", "update"):
            program_id = full["programId"] if full else None
            task_id = full["taskId"] if full else None

        elif op == "delete":
            if before and "programId" in before and "taskId" in before:
                program_id = before["programId"]
                task_id = before["taskId"]
            else:
                rb_log.warning(f"delete step: {doc_id} pre-image is not found")
                return

        if program_id is None or task_id is None:
            return

        steps_doc = await steps_col.find({"programId": program_id, "taskId": task_id}).to_list(
            length=None
        )

        if full:
            full["stepId"] = str(full.pop("_id"))

        steps_tree = self.build_step_tree(steps_doc)

        fire_and_log(
            socket_client.emit(
                f"program/task/{task_id}/change",
                {
                    "programId": str(program_id),
                    "taskId": str(task_id),
                    "step": full,
                    "stepsTree": steps_tree,
                },
            )
        )

        async with await db["client"].start_session() as s:

            async def txn(sess):
                await self.recompute_task_and_program(
                    db=db, session=sess, program_id=program_id, task_id=task_id
                )

            await s.with_transaction(
                txn,
                read_concern="snapshot",
                write_concern=WriteConcern("majority"),
                read_preference=ReadPreference.PRIMARY,
            )

    async def steps_watch_worker(self):
        """
        Steps collection의 Change Stream을 처리하는 백그라운드 작업자.
        """
        db = await get_db()

        steps_col = db["steps"]

        # Change Stream: update에도 최신 문서를 주도록
        # pre-image 쓰려면 서버/컬렉션 설정 + fullDocumentBeforeChange="required"
        pipeline = [
            {"$match": {"operationType": {"$in": ["insert", "update", "replace", "delete"]}}}
        ]

        while True:
            try:
                async with steps_col.watch(
                    pipeline,
                    full_document="updateLookup",
                    # full_document_before_change="required",  # pre-image 활성화 시 주석 해제
                    batch_size=100,
                ) as stream:
                    async for change in stream:
                        with contextlib.suppress(Exception):
                            await self._handle_step_change(db=db, change=change)
            except PyMongoError:
                rb_log.debug(f"steps_watch_worker error: {sys.exc_info()}")
                await asyncio.sleep(0.5)

    def _load_tree_from_script(self, script_path: str) -> Step:
        """
        주어진 Python 스크립트 경로에서 `tree` 변수를 가져옴
        """
        path = Path(script_path).resolve()

        if not path.exists():
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {path}")

        module_name = path.stem

        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise ImportError(f"모듈을 로드할 수 없습니다: {path}")

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        if not hasattr(module, "tree"):
            raise AttributeError(f"{path} 안에 `tree` 변수가 없습니다.")

        return cast(Step, module.tree)

    def get_executor_state(self):
        return self.script_executor.get_all_states()

    async def send_executor_state(self):
        while True:
            try:
                fire_and_log(socket_client.emit("executor/state", self.get_executor_state()))
            except Exception as e:
                rb_log.error(f"send_executor_state error: {e}")
            await asyncio.sleep(1)

    async def preview_start_program(self, request: Request_Preview_Start_ProgramPD):
        request_dict = t_to_dict(request)
        scripts = request_dict["scripts"]

        for script in scripts:
            task_id = script["taskId"]
            repeat_count = script["repeatCount"]
            steps_tree = script["steps"]

            tree = self.build_tree_from_client(steps_tree, task_id)

            self.script_executor.start(task_id, tree, repeat_count)

        return {
            "status": "success",
        }

    async def preview_stop_program(self, request: Request_Preview_Stop_ProgramPD):
        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        for task_id in task_ids:
            self.script_executor.stop(task_id)

        return {
            "status": "success",
        }

    async def preview_pause_program(self, request: Request_Preview_Stop_ProgramPD):
        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        for task_id in task_ids:
            self.script_executor.pause(task_id)

        return {
            "status": "success",
        }

    async def preview_resume_program(self, request: Request_Preview_Stop_ProgramPD):
        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        for task_id in task_ids:
            self.script_executor.resume(task_id)

        return {
            "status": "success",
        }

    async def start_program(self, request: Request_Program_ExecutionPD, db: MongoDB):
        """
        ScriptExecutor를 시작하는 함수.
        """

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]

        program_col = db["programs"]
        tasks_col = db["tasks"]

        program_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not program_doc:
            raise HTTPException(status_code=404, detail="Program not found")

        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        task_ids = [str(t["_id"]) for t in tasks_docs]

        await self.start_tasks(
            request=Request_Tasks_ExecutionPD(
                taskIds=task_ids, repeatCount=program_doc["repeatCnt"]
            ),
            db=db,
        )

        return {
            "status": "success",
        }

    async def stop_program(self, request: Request_Program_ExecutionPD, db: MongoDB):
        """
        Program을 중지하는 함수.
        """

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        for task in tasks_docs:
            self.stop_script(task["scriptName"])

        return {
            "status": "success",
        }

    async def pause_program(self, request: Request_Program_ExecutionPD, db: MongoDB):
        """
        Program을 일시정지하는 함수.
        """

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        for task in tasks_docs:
            self.pause_script(task["scriptName"])

        return {
            "status": "success",
        }

    async def resume_program(self, request: Request_Program_ExecutionPD, db: MongoDB):
        """
        Program을 재개하는 함수.
        """

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        for task in tasks_docs:
            self.resume_script(task["scriptName"])

        return {
            "status": "success",
        }

    async def start_tasks(self, request: Request_Tasks_ExecutionPD, db: MongoDB):
        """
        Tasks를 시작하는 함수.
        """

        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]
        repeat_count = request_dict["repeatCount"] or 1

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find(
            {"_id": {"$in": [ObjectId(tid) for tid in task_ids]}}
        ).to_list(length=None)
        print("tasks_docs >>>", tasks_docs, task_ids, flush=True)
        for task_doc in tasks_docs:
            script_name = task_doc["scriptName"]
            script_path = task_doc["scriptPath"]
            extension = task_doc["extension"]

            tree = self._load_tree_from_script(f"{script_path}/{script_name}.{extension}")

            print("tree >>>", tree, flush=True)

            self.script_executor.start(script_name, tree, repeat_count)

        return {
            "status": "success",
        }

    async def stop_tasks(self, request: Request_Tasks_ExecutionPD, db: MongoDB):
        """
        Tasks를 중지하는 함수.
        """

        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find(
            {"_id": {"$in": [ObjectId(tid) for tid in task_ids]}}
        ).to_list(length=None)

        for task_doc in tasks_docs:
            self.stop_script(task_doc["scriptName"])

        return {
            "status": "success",
        }

    async def pause_tasks(self, request: Request_Tasks_ExecutionPD, db: MongoDB):
        """
        Tasks를 일시정지하는 함수.
        """

        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find(
            {"_id": {"$in": [ObjectId(tid) for tid in task_ids]}}
        ).to_list(length=None)

        for task_doc in tasks_docs:
            self.pause_script(task_doc["scriptName"])

        return {
            "status": "success",
        }

    async def resume_tasks(self, request: Request_Tasks_ExecutionPD, db: MongoDB):
        """
        Tasks를 재개하는 함수.
        """

        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find(
            {"_id": {"$in": [ObjectId(tid) for tid in task_ids]}}
        ).to_list(length=None)

        for task_doc in tasks_docs:
            self.resume_script(task_doc["scriptName"])

        return {
            "status": "success",
        }

    def stop_script(self, script_name: str):
        """
        Script을 중지하는 함수.
        """

        self.script_executor.stop(script_name)

        return {
            "status": "success",
        }

    def pause_script(self, script_name: str):
        """
        Script을 일시정지하는 함수.
        """

        self.script_executor.pause(script_name)

        return {
            "status": "success",
        }

    def resume_script(self, script_name: str):
        """
        Script을 재개하는 함수.
        """

        self.script_executor.resume(script_name)

        return {
            "status": "success",
        }
