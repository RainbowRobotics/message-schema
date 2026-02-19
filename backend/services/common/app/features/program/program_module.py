import asyncio
import contextlib
import importlib.util
import os
import sys
from collections import defaultdict
from collections.abc import (
    Iterable,
)
from datetime import (
    UTC,
    datetime,
)
from pathlib import Path
from typing import Any, Literal, cast

from bson import ObjectId
from fastapi import HTTPException
from motor.motor_asyncio import AsyncIOMotorClientSession
from pymongo import ReadPreference, ReturnDocument, WriteConcern
from pymongo.errors import PyMongoError
from pymongo.operations import UpdateOne
from rb_database import MongoDB, get_db
from rb_database.utils import make_check_include_query, make_check_search_text_query
from rb_flat_buffers.flow_manager.RB_Flow_Manager_ProgramState import (
    RB_Flow_Manager_ProgramState as RB_Flow_Manager_ProgramState_FB,
)
from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller
from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.schema import MakeProcessArgs, RB_Flow_Manager_ProgramState
from rb_flow_manager.step import Step
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_resources.file import read_json_file
from rb_sdk.manipulate import RBManipulateSDK
from rb_utils.asyncio_helper import fire_and_log
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError, ZenohTransportError

from app.features.info.info_module import InfoService
from app.features.info.info_schema import RobotInfo
from app.socket.socket_client import socket_client

from .program_schema import (
    MainTaskBegin,
    PlayState,
    Request_Clone_ProgramPD,
    Request_Create_Multiple_StepPD,
    Request_Create_Multiple_TaskPD,
    Request_Create_ProgramPD,
    Request_Create_TaskPD,
    Request_Delete_StepsPD,
    Request_Delete_TasksPD,
    Request_Get_Script_ContextPD,
    Request_Load_ProgramPD,
    Request_Preview_Start_ProgramPD,
    Request_Preview_Stop_ProgramPD,
    Request_Program_Dialog,
    Request_Program_ExecutionPD,
    Request_Program_Log,
    Request_Tasks_ExecutionPD,
    Request_Update_Multiple_TaskPD,
    Request_Update_ProgramPD,
    Request_Update_StepStatePD,
    Response_Get_Task_ListPD,
    TaskType,
)

_executor: ScriptExecutor | None = None
_zenoh_controller: Zenoh_Controller | None = None


rb_manipulate_sdk = RBManipulateSDK()

def _get_zenoh_controller() -> Zenoh_Controller:
    global _zenoh_controller
    if _zenoh_controller is None:
        _zenoh_controller = Zenoh_Controller()
    return _zenoh_controller


def _get_executor() -> ScriptExecutor:
    global _executor
    if _executor is None:
        _executor = ScriptExecutor(controller=_get_zenoh_controller())
    return _executor


zenoh_client = ZenohClient()

info_service = InfoService()


class ProgramService(BaseService):
    def __init__(self) -> None:
        self._robot_models: dict[str, Any] = read_json_file("data", "robot_models.json")

        self._play_state: PlayState = PlayState.IDLE
        self._task_play_state: dict[str, PlayState] = {
            robot_model: PlayState.IDLE for robot_model in
             self._robot_models
        }

        self._step_mode = False

        self.script_executor = _get_executor()
        self._script_base_path = Path("/app/data/common/scripts")

    async def call_resume_or_pause(
        self,
        *,
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
            self.script_executor.pause_all()

            res_manipulate_resume_or_pause = zenoh_client.query_one(
                "*/call_pause",
                flatbuffer_req_obj=req_manipulate_resume_or_pause,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=256,
            )
        else:
            self.script_executor.resume_all()
            res_manipulate_resume_or_pause = zenoh_client.query_one(
                "*/call_resume",
                flatbuffer_req_obj=req_manipulate_resume_or_pause,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=2,
            )

        print("res_manipulate_resume_or_pause >>", res_manipulate_resume_or_pause, flush=True)

        # if program_id:
        #     await self.update_program_state(program_id=program_id, state=state, db=db)
        # if flow_id:
        #     await self.update_flow_state(flow_id=flow_id, state=state, db=db)

        return res_manipulate_resume_or_pause

    def call_stop(self):
        manipulate_req = Request_MotionHaltT()

        self.script_executor.stop_all()

        res_manipulate_halt = zenoh_client.query_one(
            "*/call_halt",
            flatbuffer_req_obj=manipulate_req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        return {
            "manipulateReturnValue": (
                res_manipulate_halt["obj_payload"].returnValue
                if res_manipulate_halt and res_manipulate_halt.get("obj_payload")
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

    def convert_state_to_string(
        self,
        state: int,
    ) -> RB_Flow_Manager_ProgramState:
        return {
            RB_Flow_Manager_ProgramState_FB.IDLE: RB_Flow_Manager_ProgramState.IDLE,
            RB_Flow_Manager_ProgramState_FB.RUNNING: RB_Flow_Manager_ProgramState.RUNNING,
            RB_Flow_Manager_ProgramState_FB.PAUSED: RB_Flow_Manager_ProgramState.PAUSED,
            RB_Flow_Manager_ProgramState_FB.STOPPED: RB_Flow_Manager_ProgramState.STOPPED,
            RB_Flow_Manager_ProgramState_FB.WAITING: RB_Flow_Manager_ProgramState.WAITING,
            RB_Flow_Manager_ProgramState_FB.ERROR: RB_Flow_Manager_ProgramState.ERROR,
            RB_Flow_Manager_ProgramState_FB.COMPLETED: RB_Flow_Manager_ProgramState.COMPLETED,
            RB_Flow_Manager_ProgramState_FB.AFTER_COMPLETED: RB_Flow_Manager_ProgramState.POST_START,
        }[state]

    def get_play_state(self):
        all_states = self.script_executor.get_all_states()

        return {
            "states": all_states,
        }

    def update_executor_state(self, state: int, error: str | None = None) -> None:
        str_state = self.convert_state_to_string(state)

        if str_state == RB_Flow_Manager_ProgramState.RUNNING:
            self._play_state = PlayState.PLAY
        elif str_state == RB_Flow_Manager_ProgramState.WAITING:
            self._play_state = PlayState.WAITING
        elif str_state == RB_Flow_Manager_ProgramState.POST_START:
            self._play_state = PlayState.POST_START
        elif str_state == RB_Flow_Manager_ProgramState.PAUSED:
            self._play_state = PlayState.PAUSE
        elif (
            str_state == RB_Flow_Manager_ProgramState.STOPPED
            or str_state == RB_Flow_Manager_ProgramState.ERROR
        ) or str_state == RB_Flow_Manager_ProgramState.COMPLETED:
            self._play_state = PlayState.STOP
        elif str_state == RB_Flow_Manager_ProgramState.IDLE:
            self._play_state = PlayState.IDLE

        all_states = self.script_executor.get_all_states()

        step_mode = False

        for state in all_states.values():
            if state.get("step_mode", False):
                step_mode = True
                break

        fire_and_log(
            socket_client.emit(
                "program/play_state", {"playState": self._play_state, "stepMode": step_mode, "error": error}
            )
        )

    async def load_program(self, *, request: Request_Load_ProgramPD, db: MongoDB):
        """
        Program을 조회하는 함수.
        """

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]

        program_col = db["programs"]
        robot_info_col = db["robot_info"]

        program_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not program_doc:
            raise HTTPException(status_code=404, detail="Program not found")

        robot_info_doc: dict[str, Any] | None = await robot_info_col.find_one(
            filter={},
            projection={"_id": 1},
            sort=[("_id", 1)],
        )

        if robot_info_doc is None:
            await robot_info_col.insert_one(dict(RobotInfo(programId=program_id)))

        if robot_info_doc is not None and "_id" in robot_info_doc:
            await robot_info_col.update_one(
                {"_id": robot_info_doc["_id"]},
                {"$set": {"programId": program_id}},
            )

        return await self.get_program_info(program_id=program_id, db=db)

    async def get_task_list(
        self,
        *,
        task_type: TaskType | None = None,
        robot_model: str,
        search_text: str | None = None,
        db: MongoDB,
    ) -> Response_Get_Task_ListPD:
        """
        Task 목록을 조회하는 함수.
        """
        if not robot_model:
            raise HTTPException(status_code=400, detail="Robot model is required")

        tasks_col = db["tasks"]
        query: dict[str, Any] = {"robotModel": robot_model}
        if task_type:
            query["type"] = task_type

        if search_text:
            search_text = search_text.replace(f"{robot_model}_", "")
            query = make_check_search_text_query("scriptName", search_text, query=query)

        tasks_docs = await tasks_col.find(query).to_list(length=None)

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat()
            doc["updatedAt"] = doc["updatedAt"].isoformat()

        return {
            "tasks": tasks_docs,
        }

    def build_step_tree(self, steps: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
        """
        평면 steps를 트리로 변환한다.
        """

        nodes: dict[str, dict] = {}
        for step in steps:
            step_id = step.get("stepId") or step.get("_id")
            step["stepId"] = str(step_id)
            step["createdAt"] = step["createdAt"].isoformat() if isinstance(step["createdAt"], datetime) else step["createdAt"]
            step["updatedAt"] = step["updatedAt"].isoformat() if isinstance(step["updatedAt"], datetime) else step["updatedAt"]
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
        step_doc["createdAt"] = step_doc["createdAt"].isoformat()
        step_doc["updatedAt"] = step_doc["updatedAt"].isoformat()

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
            doc["createdAt"] = doc["createdAt"].isoformat() if isinstance(doc["createdAt"], datetime) else doc["createdAt"]
            doc["updatedAt"] = doc["updatedAt"].isoformat() if isinstance(doc["updatedAt"], datetime) else doc["updatedAt"]

        return {
            "steps": steps_docs,
            "stepTree": self.build_step_tree(steps_docs),
        }

    async def upsert_steps(self, *, request: Request_Create_Multiple_StepPD, db: MongoDB):
        req = Request_Create_Multiple_StepPD.model_validate(t_to_dict(request)).model_dump()
        now = datetime.now(UTC)

        steps_col = db["steps"]
        tasks_col = db["tasks"]
        program_col = db["programs"]

        raw_steps: list[dict] = [t_to_dict(s) for s in req["steps"]]

        checked_actual_task_ids: set[str] = set()  # 이미 검증된 실제 ObjectId들
        record_find_program_ids: set[str] = set()
        task_id_map: dict[str, str] = {}  # 입력 taskId -> 실제 ObjectId 매핑

        for s in raw_steps:
            tid, pid, children = s.get("taskId"), s.get("programId"), s.get("children")

            if tid:
                # 이미 매핑된 경우 스킵
                if tid in task_id_map:
                    continue

                # ObjectId인지 확인
                if ObjectId.is_valid(tid):
                    doc = await tasks_col.find_one({"_id": ObjectId(tid)})
                    if not doc:
                        raise HTTPException(400, f"Invalid taskId: {tid}")
                    actual_task_id = str(doc["_id"])
                else:
                    # ObjectId가 아니면 rawTaskId로 찾기
                    doc = await tasks_col.find_one({"rawTaskId": tid})
                    if not doc:
                        raise HTTPException(400, f"Invalid taskId: {tid}")
                    actual_task_id = str(doc["_id"])

                # 중복 검증: 실제 ObjectId가 이미 처리되었는지 확인
                if actual_task_id in checked_actual_task_ids:
                    # 이미 처리된 task - 매핑만 추가
                    task_id_map[tid] = actual_task_id
                    continue

                # 새로운 task
                task_id_map[tid] = actual_task_id
                checked_actual_task_ids.add(actual_task_id)

            if pid and pid not in record_find_program_ids:
                doc = await program_col.find_one({"_id": ObjectId(pid)})
                if not doc:
                    raise HTTPException(400, f"Invalid programId: {pid}")
                record_find_program_ids.add(pid)

            if children:
                raise HTTPException(400, "Children is not allowed in upsert steps")

        local_to_oid: dict[str, ObjectId] = {}
        prepped_steps: list[dict] = []

        def pick_local_key(s: dict) -> str | None:
            sid = s.get("stepId")
            if sid and not ObjectId.is_valid(sid):
                return str(sid)
            return None

        for s in raw_steps:
            sid = s.get("stepId")
            s.pop("createdAt", None)
            s.pop("updatedAt", None)

            if sid and ObjectId.is_valid(sid):
                oid = ObjectId(sid)
            else:
                local = pick_local_key(s)
                if local is None:
                    raise HTTPException(400, "cannot derive local key for new step")
                oid = local_to_oid.setdefault(local, ObjectId())

            if s.get("varName") and s.get("varType") is None:
                raise HTTPException(400, "If varName is set, varType is required")
            elif s.get("varType") and s.get("varName") is None:
                raise HTTPException(400, "If varType is set, varName is required")

            # taskId를 실제 ObjectId로 교체
            original_task_id = s.get("taskId")
            if original_task_id and original_task_id in task_id_map:
                s["taskId"] = task_id_map[original_task_id]

            s["_id"] = oid
            s["stepId"] = str(oid)
            prepped_steps.append(s)

        def resolve_ref(val: str | None) -> str | None:
            if val is None:
                return None
            if ObjectId.is_valid(val):
                return val
            if val in local_to_oid:
                return str(local_to_oid[val])
            raise HTTPException(400, f"unknown parent/sync reference: {val}")

        for s in prepped_steps:
            s["parentStepId"] = resolve_ref(s.get("parentStepId"))
            target_step_id = s.get("targetStepId")
            if isinstance(target_step_id, str) and not target_step_id.strip():
                target_step_id = None
            s["targetStepId"] = resolve_ref(target_step_id)
            if "syncStepIds" in s and s["syncStepIds"]:
                s["syncStepIds"] = [resolve_ref(x) for x in s["syncStepIds"]]

            s["updatedAt"] = now

        client = db.client

        async with await client.start_session() as session, session.start_transaction():
            ops: list[UpdateOne] = []
            for s in prepped_steps:
                oid = s["_id"]
                body = {**s}
                body.pop("_id", None)

                ops.append(
                    UpdateOne(
                        {"_id": oid},
                        {"$set": body, "$setOnInsert": {"createdAt": now}},
                        upsert=True,
                    )
                )

            if ops:
                await steps_col.bulk_write(ops, ordered=False, session=session)

            by_task: dict[str, set[ObjectId]] = {}

            for s in prepped_steps:
                if s.get("taskId"):
                    by_task.setdefault(s["taskId"], set()).add(s["_id"])

            for tid, keep_set in by_task.items():
                await steps_col.delete_many(
                    {"taskId": tid, "_id": {"$nin": list(keep_set)}}, session=session
                )

        # 4) 후처리(트랜잭션 밖)
        for s in prepped_steps:
            tid = s.get("taskId")
            if tid:
                fire_and_log(self.write_script_context(tid, db), name="write_script_context")

        return {"ok": True, "count": len(prepped_steps)}

    async def update_all_task_step_state(
        self, *, task_id: str, state: RB_Flow_Manager_ProgramState
    ):
        if state in [
            RB_Flow_Manager_ProgramState.RUNNING,
            RB_Flow_Manager_ProgramState.WAITING,
            RB_Flow_Manager_ProgramState.COMPLETED,
        ]:
            self._task_play_state[task_id] = PlayState.PLAY
        elif state == RB_Flow_Manager_ProgramState.PAUSED:
            self._task_play_state[task_id] = PlayState.PAUSE
        elif state in [RB_Flow_Manager_ProgramState.STOPPED, RB_Flow_Manager_ProgramState.ERROR]:
            self._task_play_state[task_id] = PlayState.STOP
        elif state == RB_Flow_Manager_ProgramState.IDLE:
            self._task_play_state[task_id] = PlayState.IDLE



        if state == RB_Flow_Manager_ProgramState.IDLE:
            fire_and_log(
                socket_client.emit(
                    f"program/task/{task_id}/ended",
                    {
                        "taskId": task_id,
                        "state": state,
                    },
                ),
                name="emit_task_ended",
            )

            fire_and_log(
                socket_client.emit(
                    f"program/task/{task_id}/state",
                    {
                        "state": state,
                        "taskId": task_id,
                        "robotModel": self.script_executor.get_state(task_id).get("robot_model", "*"),
                        "stepMode": False,
                    },
                ),
                name="emit_task_state",
            )

    async def update_step_state(self, *, request: Request_Update_StepStatePD):
        request_dict = t_to_dict(request)

        step_id = request_dict["stepId"]
        task_id = request_dict["taskId"]
        state = request_dict["state"]
        error_value: str | None = request_dict.get("error")

        execute_states = self.script_executor.get_all_states()

        if execute_states is not None and task_id in execute_states:
            execute_state = execute_states.get(task_id)

            fire_and_log(
                socket_client.emit(
                    f"program/{execute_state.get('robot_model')}/change-variables",
                    {
                        "taskId": task_id,
                        "variables": execute_state.get("variables", {}),
                    },
                ),
            )

            fire_and_log(
                socket_client.emit(
                    f"program/task/{task_id}/state", {
                        "state": execute_state.get("state"),
                        "taskId": task_id,
                        "robotModel": execute_state.get("robot_model"),
                        "stepMode": execute_state.get("step_mode"),
                    }
                )
            )



        fire_and_log(
            socket_client.emit(
                f"program/task/{task_id}/update_state",
                {
                    "stepId": step_id,
                    "state": state,
                    "error": error_value,
                },
            ),
            name="emit_step_update_state",
        )

        if not ObjectId.is_valid(step_id):
            return

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

    def get_ctx_sub_task_list(self, task_id: str):
        all_states = self.get_executor_state() or {}

        task_state = dict(all_states.get(task_id, {}) or {})
        sub_task_list = list(task_state.get("sub_task_list", []) or [])

        res_sub_task_list = []
        for st in sub_task_list:
            st = dict(st or {})  # 원소가 dict면 얕은 복사
            res_sub_task_list.append({
                "taskId": st.get("task_id"),
                "subTaskType": st.get("sub_task_type"),
            })

        return {"subTaskList": res_sub_task_list}

    def update_sub_task_state(self, *, task_id: str):
        """ 서브 태스크 상태 업데이트 함수 """

        res = self.get_ctx_sub_task_list(task_id)

        fire_and_log(
            socket_client.emit(
                f"program/task/{task_id}/sub-task-list",
                {
                    "subTaskList": res.get("subTaskList", []),
                },
            ),
        )

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

        main_task_doc = await task_col.find_one({"_id": ObjectId(task_id)})
        sub_task_docs = await task_col.find({"parentTaskId": task_id, "type": TaskType.SUB}).to_list(length=None)
        event_sub_task_docs = await task_col.find({"parentTaskId": task_id, "type": TaskType.EVENT_SUB}).to_list(length=None)



        if not main_task_doc:
            raise HTTPException(status_code=400, detail=f"Invalid taskId: {task_id}")

        program_doc = await program_col.find_one({"_id": ObjectId(main_task_doc["programId"])})

        if not program_doc:
            raise HTTPException(
                status_code=400, detail=f"Invalid programId: {main_task_doc['programId']}"
            )

        steps_docs = await steps_col.find({"taskId": task_id}).to_list(length=None)
        sub_steps_docs: dict[str, list[Step]] = defaultdict(list)
        event_sub_steps_docs: dict[str, list[Step]] = defaultdict(list)

        for sub_task_doc in sub_task_docs:
            sub_steps_docs[str(sub_task_doc["_id"])] = self.build_step_tree(await steps_col.find({"taskId": str(sub_task_doc["_id"])}).to_list(length=None))

        for event_sub_task_doc in event_sub_task_docs:
            event_sub_steps_docs[str(event_sub_task_doc["_id"])] = self.build_step_tree(await steps_col.find({"taskId": str(event_sub_task_doc["_id"])}).to_list(length=None))

        steps_tree = self.build_step_tree(steps_docs)

        normal_steps_tree = []
        post_steps_tree = []

        for step in steps_tree:
            if step.get("method", None) == "ActionPost":
                post_steps_tree.append(step)
            else:
                normal_steps_tree.append(step)

        if len(post_steps_tree) > 1:
            raise HTTPException(status_code=400, detail=f"ActionPost step must be only one: {task_id}")

        script_context = self.build_script_context(
            normal_steps_tree,
            task_id,
            main_task_doc["scriptName"],
            program_doc["repeatCnt"],
            robot_model=main_task_doc["robotModel"],
            begin=main_task_doc.get("begin", None),
            post_tree=post_steps_tree,
            sub_task_steps=sub_steps_docs,
            event_sub_task_steps=event_sub_steps_docs,
        )

        os.makedirs(main_task_doc["scriptPath"], exist_ok=True)

        with open(
            f"{main_task_doc['scriptPath']}/{main_task_doc['scriptName']}.{main_task_doc['extension']}",
            "w",
            encoding="utf-8",
        ) as f:
            return f.write(script_context)

    def parse_step_context(self, step: dict[str, Any], depth: int = 0, *, to_string: bool = False):
        """Steps tree를 컨텍스트로 변환하는 함수."""

        step_obj = Step.from_dict(step)
        return step_obj.to_py_string(depth) if to_string else step_obj

    def build_tree_from_client(self, steps_tree: list[dict[str, Any]], task_id: str, robot_model: str | None = None, category: str | None = None, begin: dict[str, Any] | None = None):
        """
        프론트에서 받은 JSON(dict)을 실제 Step / RepeatStep 트리 객체로 변환하는 함수.
        """
        root_func_name: str | None = None
        root_args: dict[str, Any] = {}

        if begin is not None and robot_model is not None:
            root_func_name = f"rb_{category}_sdk.program.set_begin"
            root_args = {
                "robot_model": robot_model,
                "position": begin.get("position", None),
                "is_enable": begin.get("is_enable", True),
                "speed_ratio": begin.get("speed_ratio", None),
            }

        return Step(
            step_id=task_id,
            name=task_id,
            func_name=root_func_name,
            args=root_args,
            children=[self.parse_step_context(step, depth=8) for step in steps_tree],
        )

    # def build_sub_script_context(self, steps_tree: list[dict[str, Any]], task_id: str):
    #     """
    #     Sub Task의 스크립트 컨텍스트를 조회하는 함수.
    #     """
    #     return self.build_script_context(steps_tree, task_id, task_id, 1)

    def build_script_context(
        self,
        steps_tree: list[Step],
        task_id: str,
        script_name: str,
        repeat_count: int,
        *,
        robot_model: str | None = None,
        begin: MainTaskBegin | None = None,
        sub_task_steps: dict[str, list[Step]] | None = None,
        event_sub_task_steps: dict[str, list[Step]] | None = None,
        post_tree: list[dict[str, Any]] | None = None,
    ):
        """
        Steps tree를 컨텍스트로 변환하는 함수.
        """
        category = self._robot_models.get(robot_model or "", {}).get("be_service", None)

        body_context = "\n".join(
            self.parse_step_context(step, depth=8, to_string=True) for step in steps_tree
        )

        post_body_context = "\n".join(
            self.parse_step_context(step, depth=0, to_string=True) for step in post_tree
        ) if post_tree else ""

        # Header
        header_context = f'""" {script_name} """\n'
        header_context = "from rb_flow_manager.executor import ScriptExecutor\n"
        header_context += "from rb_flow_manager.schema import MakeProcessArgs\n"
        header_context += "from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep, SyncStep, CallEventStep\n"
        header_context += "from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller\n"

        # Context variables
        context_part = "\nsub_tree_list: list[Step] = []\n"
        context_part += "event_sub_tree_list: list[Step] = []\n\n"

        # Begin arguments (optional)
        func_part = ""
        args_part = ""

        begin_dict = t_to_dict(begin) if begin is not None else None

        if begin_dict is not None and robot_model is not None:
            func_part = f"    func_name='rb_{category}_sdk.program.set_begin',\n"
            args_part = (
                f"    args={{\n"
                f"        'robot_model': '{robot_model}',\n"
                f"        'position': {begin_dict.get('position', None)},\n"
                f"        'is_enable': {begin_dict.get('is_enable', True)},\n"
                f"        'speed_ratio': {begin_dict.get('speed_ratio', None)},\n"
                f"    }},\n"
            )

        # Main tree
        root_block = (
            "\ntree = Step(\n"
            f"    step_id='{task_id}',\n"
            f"    name='{script_name}',\n"
            f"{func_part}"
            f"{args_part}"
            "    children=[\n"
            f"{body_context}\n"
            "    ],\n"
            ")\n"
        )

        # Sub task trees
        sub_task_root_block = ""
        if sub_task_steps is not None:
            for steps in sub_task_steps.values():
                sub_task_body_context = "\n".join(
                    self.parse_step_context(step, depth=4, to_string=True) for step in steps
                )

                sub_task_root_block += (
                    "\n\nsub_tree_list.append(\n"
                    f"{sub_task_body_context}\n"
                    ")\n"
                )

        # Event sub task trees
        event_sub_task_root_block = ""
        if event_sub_task_steps is not None:
            for steps in event_sub_task_steps.values():
                event_sub_task_body_context = "\n".join(
                    self.parse_step_context(step, depth=4, to_string=True) for step in steps
                )

                event_sub_task_root_block += (
                    "\n\nevent_sub_tree_list.append(\n"
                    f"{event_sub_task_body_context}\n"
                    ")\n"
                )

        # Post tree (if exists)
        post_root_block = (
            f"\n\npost_tree = {post_body_context}\n"
        ) if post_body_context else ""

        # Footer - main execution block
        footer_context = '\n\nif __name__ == "__main__":\n\n'
        footer_context += "    zenoh_controller = Zenoh_Controller()\n"
        footer_context += "    executor = ScriptExecutor(controller=zenoh_controller)\n\n"

        footer_context += "    processes = [\n"
        footer_context += "        MakeProcessArgs(\n"
        footer_context += f"            process_id='{script_name}',\n"
        footer_context += "            step=tree,\n"
        footer_context += f"            repeat_count={repeat_count},\n"
        footer_context += f"            robot_model={repr(robot_model)},\n"
        footer_context += f"            category={repr(category)},\n"
        footer_context += "            event_sub_tree_list=event_sub_tree_list,\n"
        footer_context += "        )\n"
        footer_context += "    ]\n\n"

        # Add sub tree processes (linked to main process)
        footer_context += "    for index, sub_tree in enumerate(sub_tree_list):\n"
        footer_context += "        processes.append(\n"
        footer_context += "            MakeProcessArgs(\n"
        footer_context += f"                process_id=f\"{script_name}_sub_tree_{{index}}\",\n"
        footer_context += "                step=sub_tree,\n"
        footer_context += f"                repeat_count={repeat_count},\n"
        footer_context += f"                robot_model={repr(robot_model)},\n"
        footer_context += f"                category={repr(category)},\n"
        footer_context += f"                parent_process_id='{script_name}',\n"
        footer_context += "            )\n"
        footer_context += "        )\n\n"

        footer_context += "    executor.start(processes)\n"

        return (
            header_context +
            context_part +
            root_block +
            sub_task_root_block +
            event_sub_task_root_block +
            post_root_block +
            footer_context
        )

    async def get_script_context(self, *, request: Request_Get_Script_ContextPD, db: MongoDB):
        """
        Task의 스크립트 컨텍스트를 조회하는 함수.
        """
        request_dict = t_to_dict(request)
        task_id = request_dict["taskId"]
        steps_tree = request_dict["steps"]
        sub_task_steps = request_dict.get("subTaskSteps", None)
        event_sub_task_steps = request_dict.get("eventSubTaskSteps", None)
        begin = request_dict.get("begin", None)

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

        normal_steps_tree = []
        post_steps_tree = []

        for step in steps_tree:
            if step.get("method", None) == "ActionPost":
                post_steps_tree.append(step)
            else:
                normal_steps_tree.append(step)

        script_context = self.build_script_context(
            normal_steps_tree,
            task_id,
            task_doc["scriptName"],
            program_doc["repeatCnt"],
            robot_model=task_doc["robotModel"],
            begin=begin,
            sub_task_steps=sub_task_steps,
            event_sub_task_steps=event_sub_task_steps,
            post_tree=post_steps_tree,
        )

        return {
            "context": script_context,
        }

    async def get_main_task_list(self, *, program_id: str, db: MongoDB):
        """
        Task 목록을 조회하는 함수.
        """

        tasks_col = db["tasks"]
        tasks_docs = await tasks_col.find({"programId": program_id, "type": TaskType.MAIN}).to_list(
            length=None
        )

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat()
            doc["updatedAt"] = doc["updatedAt"].isoformat()

        return {
            "tasks": tasks_docs,
        }

    async def get_sub_task_list(self, *, program_id: str, parent_task_id: str, db: MongoDB):
        """
        Sub Task 목록을 조회하는 함수.
        """

        tasks_col = db["tasks"]
        tasks_docs = (
            await tasks_col.find(
                {
                    "programId": program_id,
                    "type": {"$in": [TaskType.SUB, TaskType.EVENT_SUB]},
                    "parentTaskId": parent_task_id
                }
            )
            .sort("order", 1)
            .to_list(length=None)
        )

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat()
            doc["updatedAt"] = doc["updatedAt"].isoformat()

        return {
            "tasks": tasks_docs,
        }

    async def get_task_info(self, *, task_id: str, db: MongoDB):
        """
        Task 정보를 조회하는 함수.
        """

        tasks_col = db["tasks"]
        task_doc = await tasks_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=400, detail=f"No task found with taskId: {task_id}")

        task_doc["taskId"] = str(task_doc.pop("_id"))
        task_doc["createdAt"] = task_doc["createdAt"].isoformat()
        task_doc["updatedAt"] = task_doc["updatedAt"].isoformat()

        return {
            "task": task_doc,
        }

    async def get_task_state(self, *, task_id: str):
        """
        Task 상태를 조회하는 함수.
        """

        execute_states = self.script_executor.get_all_states()
        task_state = dict(execute_states.get(task_id, {}) or {})

        return {
            "state": task_state.get("state"),
            "taskId": task_id,
            "robotModel": task_state.get("robot_model"),
            "stepMode": task_state.get("step_mode"),
        }

    async def create_tasks(self, *, request: Request_Create_Multiple_TaskPD, db: MongoDB):
        """
        Task들을 생성하는 함수.
        """
        try:
            tasks_col = db["tasks"]
            program_col = db["programs"]

            request_dict = Request_Create_Multiple_TaskPD.model_validate(
                request
            ).model_dump()

            tasks = request_dict["tasks"]

            for task in tasks:
                parent_task_id = task.get("parentTaskId")
                if task.get("type") != TaskType.MAIN and parent_task_id is None:
                    raise HTTPException(status_code=400, detail="Sub task must have parentTaskId")

                # if task.get("type") == TaskType.MAIN and task.get("begin") is None:
                #     raise HTTPException(status_code=400, detail="Main task must have begin")

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
                {
                    "scriptName": {"$in": [task.get("scriptName") for task in tasks]},
                    "type": TaskType.MAIN,
                },
                {"scriptName": 1},
            ).to_list(length=None)

            if existed:
                exist_names = sorted({d["scriptName"] for d in existed})
                raise HTTPException(
                    status_code=409, detail=f"Already exists scriptName: {exist_names}"
                )
        except Exception as e:
            raise e

        try:
            res = await tasks_col.insert_many(tasks)

            for task, inserted_id in zip(tasks, res.inserted_ids, strict=False):
                task["taskId"] = str(inserted_id)
                task.pop("_id", None)
                task["createdAt"] = task["createdAt"].isoformat()
                task["updatedAt"] = task["updatedAt"].isoformat()

                if task["type"] == TaskType.MAIN:
                    fire_and_log(
                        self.write_script_context(task["taskId"], db), name="write_script_context"
                    )

            return { "tasks": tasks }
        except Exception as e:
            if tasks:
                await tasks_col.delete_many(
                    {"_id": {"$in": [ObjectId(task.get("taskId")) for task in tasks]}}
                )
            raise e

    async def update_tasks(self, *, request: Request_Update_Multiple_TaskPD, db: MongoDB):
        """
        Task들을 업데이트하는 함수.
        delete_on이 True면 요청에 포함되지 않은 기존 task들을 삭제합니다.
        """

        now = datetime.now(UTC)
        update_ops = []

        tasks_col = db["tasks"]
        program_col = db["programs"]

        delete_on = request.delete_on if hasattr(request, 'delete_on') else False

        request_dict = Request_Update_Multiple_TaskPD.model_validate(
                request
            ).model_dump(exclude_none=False, exclude_unset=True, exclude={"delete_on"})

        tasks = request_dict["tasks"]

        program_ids = {t["programId"] for t in tasks}

        record_find_program_ids = []
        updated_task_ids = []
        created_tasks = []
        raw_to_actual_task_id_map = {}  # raw taskId -> actual ObjectId 매핑

        for pid in program_ids:
            if pid in record_find_program_ids:
                continue

            if not await program_col.find_one({"_id": ObjectId(pid)}):
                raise HTTPException(status_code=400, detail=f"Invalid programId: {pid}")

            record_find_program_ids.append(pid)

        for task in tasks:
            parent_task_id = task.get("parentTaskId")
            if parent_task_id and not ObjectId.is_valid(parent_task_id):
                raise HTTPException(
                    status_code=400, detail=f"Invalid parentTaskId: {parent_task_id}"
                )
            elif task.get("type") != TaskType.MAIN and parent_task_id is None:
                raise HTTPException(status_code=400, detail="Sub task must have parentTaskId")

            tid = task.get("taskId")

            # taskId가 유효한 ObjectId인지 확인
            if tid and ObjectId.is_valid(tid):
                # Update 로직
                task_data = task.copy()
                task_data.pop("taskId")
                task_data["updatedAt"] = now

                update_ops.append(UpdateOne({"_id": ObjectId(tid)}, {"$set": task_data}))
                updated_task_ids.append(tid)
            else:
                # Create 로직
                task_data = task.copy()
                raw_task_id = task_data.pop("taskId", None)  # taskId를 빼서 저장
                task_data["createdAt"] = now
                task_data["updatedAt"] = now

                # ObjectId가 아닌 taskId를 rawTaskId에 저장
                if raw_task_id:
                    task_data["rawTaskId"] = raw_task_id

                created_tasks.append((raw_task_id, task_data))  # raw_task_id도 함께 저장

        # delete_on이 True면 요청에 없는 기존 task 삭제
        deleted_count = 0
        if delete_on:
            # 요청에 포함된 모든 taskId 수집
            existing_task_ids = [ObjectId(tid) for tid in updated_task_ids if ObjectId.is_valid(tid)]

            # 각 programId에 대해 삭제 작업 수행
            for pid in program_ids:
                delete_result = await tasks_col.delete_many({
                    "programId": pid,
                    "_id": {"$nin": existing_task_ids}
                })
                deleted_count += delete_result.deleted_count

        # Update 실행
        if update_ops:
            await tasks_col.bulk_write(update_ops, ordered=False)

        # Create 실행 및 매핑 생성
        created_ids = []
        if created_tasks:
            tasks_to_insert = [task_data for _, task_data in created_tasks]
            result = await tasks_col.insert_many(tasks_to_insert)
            created_ids = [str(oid) for oid in result.inserted_ids]

            # raw taskId -> actual ObjectId 매핑 생성
            for i, (raw_task_id, _) in enumerate(created_tasks):
                if raw_task_id:
                    actual_id = created_ids[i]
                    raw_to_actual_task_id_map[raw_task_id] = actual_id

        # parentTaskId가 raw ID인 경우 실제 ObjectId로 업데이트
        if raw_to_actual_task_id_map:
            parent_update_ops = []
            for created_id in created_ids:
                doc = await tasks_col.find_one({"_id": ObjectId(created_id)})
                if doc and doc.get("parentTaskId"):
                    parent_id = doc["parentTaskId"]
                    # parentTaskId가 raw_to_actual_task_id_map에 있으면 실제 ID로 교체
                    if parent_id in raw_to_actual_task_id_map:
                        actual_parent_id = raw_to_actual_task_id_map[parent_id]
                        parent_update_ops.append(
                            UpdateOne(
                                {"_id": ObjectId(created_id)},
                                {"$set": {"parentTaskId": actual_parent_id, "updatedAt": now}}
                            )
                        )

            if parent_update_ops:
                await tasks_col.bulk_write(parent_update_ops, ordered=False)

        # 결과 조회 (updated + created)
        all_ids = [ObjectId(tid) for tid in updated_task_ids] + [ObjectId(cid) for cid in created_ids]

        res = await tasks_col.find(
            {"_id": {"$in": all_ids}}
        ).to_list(length=None)

        for doc in res:
            doc["taskId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat()
            doc["updatedAt"] = doc["updatedAt"].isoformat()

        return {
            "tasks": res,
            "deletedCount": deleted_count if delete_on else 0,
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
        steps_col = db["steps"]
        robot_info_col = db["robot_info"]

        program_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not program_doc:
            robot_info_doc = await robot_info_col.find_one({}, sort=[("_id", 1)])

            if robot_info_doc:
                await robot_info_col.update_one(
                    {"programId": program_id}, {"$set": {"programId": None}}
                )

            program_doc = dict(RobotInfo(programId=program_id))

            return {
                "program": None,
                "mainTasks": None,
                "subTasks": None,
                "mainSteps": None,
            }

        program_doc["programId"] = str(program_doc.pop("_id"))
        program_doc["createdAt"] = program_doc["createdAt"].isoformat()
        program_doc["updatedAt"] = program_doc["updatedAt"].isoformat()

        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)
        tasks_docs.sort(key=lambda x: x.get("order", 0))

        for doc in tasks_docs:
            doc["taskId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat()
            doc["updatedAt"] = doc["updatedAt"].isoformat()

        type_counts = {}
        for doc in tasks_docs:
            t = doc.get("type", "MISSING")
            type_counts[t] = type_counts.get(t, 0) + 1
        print(f"📈 Type distribution: {type_counts}", flush=True)

        main_tasks_docs = [doc for doc in tasks_docs if doc["type"] == TaskType.MAIN]
        sub_tasks_docs = [doc for doc in tasks_docs if doc["type"] != TaskType.MAIN]

        main_steps: dict[str, list[dict]] = {}
        sub_steps: dict[str, list[dict[str, Any]]] = defaultdict(list)

        for doc in tasks_docs:
            steps_docs = await steps_col.find({"taskId": doc["taskId"]}).to_list(length=None)

            steps_tree = self.build_step_tree(steps_docs)

            if doc["type"] == TaskType.MAIN:
                main_steps[doc["taskId"]] = steps_tree
            elif doc.get("parentTaskId") is not None:
                sub_steps[doc["parentTaskId"]].append({
                    "taskId": doc["taskId"],
                    "steps": steps_tree,
                })


        return {
            "program": program_doc,
            "mainTasks": main_tasks_docs,
            "subTasks": sub_tasks_docs,
            "mainSteps": main_steps,
            "subSteps": sub_steps,
        }

    async def get_program_list(
        self,
        *,
        search_name: str | None = None,
        order: Literal["ASC", "DESC"] = "DESC",
        db: MongoDB,
    ):
        """
        Program 목록을 조회하는 함수.
        """
        robot_info = await info_service.get_robot_info(db=db)
        program_col = db["programs"]

        query: dict[str, Any] = {}

        if search_name:
            if len(search_name) > 100:
                raise ValueError("search_name must be less than 100 characters")

            query = make_check_search_text_query("name", search_name, query=query)

        query = make_check_include_query("robotModel", robot_info["info"]["robotModel"], query=query)

        sort_spec = [("createdAt", 1 if order == "ASC" else -1)]

        program_docs = await program_col.find(query).sort(sort_spec).to_list(length=None)

        for doc in program_docs:
            doc["programId"] = str(doc.pop("_id"))
            doc["createdAt"] = doc["createdAt"].isoformat() if isinstance(doc.get("createdAt"), datetime) else None
            doc["updatedAt"] = doc["updatedAt"].isoformat() if isinstance(doc.get("updatedAt"), datetime) else None

        return {
            "programs": program_docs,
        }

    async def create_program_and_tasks(self, *, request: Request_Create_ProgramPD, db: MongoDB):
        """
        Program과 Task를 동시에 생성하는 함수.
        """

        now = datetime.now(UTC)

        program_doc = Request_Create_ProgramPD.model_validate(request).model_dump()

        program_doc["createdAt"] = now
        program_doc["updatedAt"] = now

        robot_info = await info_service.get_robot_info(db=db)

        program_doc["robotModel"] = robot_info["info"]["robotModel"]

        components = robot_info["info"]["components"]

        if components is None or len(components) == 0:
            raise HTTPException(
                status_code=400, detail="Please enter your robot information first."
            )

        program_col = db["programs"]

        find_doc = await program_col.find_one({"name": program_doc["name"]})

        if find_doc:
            raise HTTPException(status_code=400, detail="Program already exists")

        try:
            program_res = await program_col.insert_one(program_doc)

            find_program_doc = await program_col.find_one({"_id": program_res.inserted_id})

            if find_program_doc:
                find_program_doc["programId"] = str(program_res.inserted_id)
                find_program_doc.pop("_id", None)
                find_program_doc["createdAt"] = find_program_doc["createdAt"].isoformat()
                find_program_doc["updatedAt"] = find_program_doc["updatedAt"].isoformat()

            tasks = []

            for component in components:
                name = f'{component}_{program_doc["name"]}'
                task = Request_Create_TaskPD(
                    programId=str(program_res.inserted_id),
                    robotModel=component,
                    name=name,
                    scriptName=name,
                    scriptPath="",
                    createdAt=now,
                    updatedAt=now,
                )

                tasks.append(task)

            tasks_res = await self.create_tasks(
                request=Request_Create_Multiple_TaskPD(tasks=tasks),
                db=db,
            )

            all_steps: dict[str, list[dict]] = {}
            tasks = tasks_res.get("tasks", [])

            for task in tasks:
                all_steps[task["taskId"]] = []

            return {
                "program": find_program_doc,
                "mainTasks": [task for task in tasks if task["type"] == TaskType.MAIN],
                "subTasks": [task for task in tasks if task["type"] != TaskType.MAIN],
                "mainSteps": all_steps,
                "subSteps": None,
            }

        except Exception as e:
            if program_res:
                await program_col.delete_one({"_id": program_res.inserted_id})

            raise e

    async def update_program(self, *, request: Request_Update_ProgramPD, db: MongoDB):
        """
        Program과 Task를 동시에 업데이트하는 함수.
        """

        try:
            now = datetime.now(UTC)

            program_doc = Request_Update_ProgramPD.model_validate(t_to_dict(request)).model_dump(
                exclude_none=True, exclude_unset=True
            )
            program_doc["updatedAt"] = now

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
            program_res["createdAt"] = program_res["createdAt"].isoformat()
            program_res["updatedAt"] = program_res["updatedAt"].isoformat()

            return {
                "program": program_res,
            }
        except Exception as e:
            raise e

    async def clone_program(self, *, request: Request_Clone_ProgramPD, db: MongoDB):
        """
        Program을 복제하는 함수.
        """

        now = datetime.now(UTC)

        program_col = db["programs"]
        tasks_col = db["tasks"]
        steps_col = db["steps"]

        request_dict = t_to_dict(request)
        program_id = request_dict["programId"]
        new_name = request_dict["newName"]

        find_doc = await program_col.find_one({"_id": ObjectId(program_id)})

        if not find_doc:
            raise HTTPException(status_code=400, detail="Program not found")

        tasks_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        new_program_id: str | None = None

        try:
            clone_program = find_doc.copy()
            clone_program.pop("_id")
            clone_program["name"] = new_name
            clone_program["createdAt"] = now
            clone_program["updatedAt"] = now

            find_same_name_program = await program_col.find_one({"name": new_name})

            if find_same_name_program:
                raise HTTPException(
                    status_code=400, detail="Program with the same name already exists"
                )

            program_insert_res = await program_col.insert_one(clone_program)
            new_program_id = str(program_insert_res.inserted_id)
        except Exception as e:
            raise e

        tasks_id_map: dict[str, str] = {}
        steps_id_map: dict[str, str] = {}
        new_step_ids: list[str] = []

        try:
            # Task 복제
            for task in tasks_docs:
                pre_task_id = str(task.pop("_id"))
                task["programId"] = new_program_id
                task["scriptName"] = task["scriptName"].replace(find_doc["name"], new_name)
                task["createdAt"] = now
                task["updatedAt"] = now
                new_task_res = await tasks_col.insert_one(task)

                new_task_id = str(new_task_res.inserted_id)
                tasks_id_map[pre_task_id] = new_task_id

                if task["type"] == TaskType.MAIN:
                    fire_and_log(
                        self.write_script_context(new_task_id, db), name="write_script_context"
                    )

            # 모든 Step 문서 수집
            all_steps_to_insert = []
            old_step_id_to_parent = {}  # old_step_id -> old_parent_step_id 매핑
            old_step_id_to_target = {}  # old_step_id -> old_target_step_id 매핑

            for old_task_id, new_task_id in tasks_id_map.items():
                steps_docs = await steps_col.find({"taskId": old_task_id}).to_list(length=None)

                for step in steps_docs:
                    old_step_id = str(step.pop("_id"))
                    old_parent_step_id = step.get("parentStepId")
                    old_target_step_id = step.get("targetStepId")

                    step["taskId"] = new_task_id
                    step["createdAt"] = now
                    step["updatedAt"] = now

                    all_steps_to_insert.append((old_step_id, step))

                    # parentStepId가 있으면 매핑 저장
                    if old_parent_step_id:
                        old_step_id_to_parent[old_step_id] = old_parent_step_id

                    # targetStepId가 있으면 매핑 저장
                    if old_target_step_id:
                        old_step_id_to_target[old_step_id] = old_target_step_id

            # Step 일괄 삽입
            if all_steps_to_insert:
                steps_to_insert = [step_data for _, step_data in all_steps_to_insert]
                insert_result = await steps_col.insert_many(steps_to_insert)

                # steps_id_map 생성
                for i, (old_step_id, _) in enumerate(all_steps_to_insert):
                    new_step_id = str(insert_result.inserted_ids[i])
                    steps_id_map[old_step_id] = new_step_id
                    new_step_ids.append(new_step_id)

            # parentStepId, targetStepId 일괄 업데이트
            update_ops = []
            for i, (old_step_id, _) in enumerate(all_steps_to_insert):
                new_step_id = new_step_ids[i]
                update_fields = {}

                # parentStepId 업데이트
                if old_step_id in old_step_id_to_parent:
                    old_parent_step_id = old_step_id_to_parent[old_step_id]

                    if old_parent_step_id in steps_id_map:
                        new_parent_step_id = steps_id_map[old_parent_step_id]
                        update_fields["parentStepId"] = new_parent_step_id

                # targetStepId 업데이트
                if old_step_id in old_step_id_to_target:
                    old_target_step_id = old_step_id_to_target[old_step_id]

                    if old_target_step_id in steps_id_map:
                        new_target_step_id = steps_id_map[old_target_step_id]
                        update_fields["targetStepId"] = new_target_step_id

                # 업데이트할 필드가 있으면 bulk operation 추가
                if update_fields:
                    update_fields["updatedAt"] = now
                    update_ops.append(
                        UpdateOne(
                            {"_id": ObjectId(new_step_id)},
                            {"$set": update_fields}
                        )
                    )

            if update_ops:
                await steps_col.bulk_write(update_ops, ordered=False)

        except Exception as e:
            if new_program_id:
                await program_col.delete_one({"_id": ObjectId(new_program_id)})

            if tasks_id_map:
                await tasks_col.delete_many({
                    "_id": {
                        "$in": [ObjectId(tid) for tid in tasks_id_map.values()]
                        }
                    }
                )

            if new_step_ids:
                await steps_col.delete_many({
                    "_id": {
                        "$in": [ObjectId(sid) for sid in new_step_ids]
                        }
                    }
                )

            raise e

        return {
            "new_program_id": new_program_id,
            "tasks_id_map": tasks_id_map,
            "steps_id_map": steps_id_map,
        }

    async def delete_program(self, *, program_id: str, db: MongoDB):
        """
        Program을 삭제하는 함수.
        """

        program_col = db["programs"]
        tasks_col = db["tasks"]
        steps_col = db["steps"]

        task_docs = await tasks_col.find({"programId": program_id}).to_list(length=None)

        program_deleted_res = await program_col.delete_one({"_id": ObjectId(program_id)})
        task_deleted_res = await tasks_col.delete_many({"programId": program_id})
        step_deleted_res = await steps_col.delete_many({"programId": program_id})

        for task in task_docs:
            script_full_path = f"{task['scriptPath']}/{task['scriptName']}.{task['extension']}"

            if os.path.exists(script_full_path):
                os.remove(script_full_path)

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

    def _load_event_sub_trees_from_script(self, script_path: str) -> list[Step]:
        path = Path(script_path).resolve()

        if not path.exists():
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {path}")

        module_name = f"{path.stem}_event_sub"
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise ImportError(f"모듈을 로드할 수 없습니다: {path}")

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        event_sub_tree_list = getattr(module, "event_sub_tree_list", [])
        if not isinstance(event_sub_tree_list, list):
            return []

        return cast(list[Step], event_sub_tree_list)

    def get_executor_variables(self, robot_model: str):
        """
        Executor의 모든 변수를 조회하는 함수.
        """

        execute_states = self.script_executor.get_all_states()

        variables: dict[str, Any] = {}

        for execute_state in execute_states.values():
            if execute_state.get("robot_model") == robot_model:
                variables.update(execute_state.get("variables", {}))

        return {
            "variables": variables,
        }

    def get_executor_state(self):
        return self.script_executor.get_all_states()

    async def send_executor_state(self):
        while True:
            try:
                fire_and_log(socket_client.emit("executor/state", self.get_executor_state()))
            except Exception as e:
                rb_log.error(f"send_executor_state error: {e}")
            await asyncio.sleep(1)

    async def preview_start_program(self, request: Request_Preview_Start_ProgramPD, db: MongoDB):
        request_dict = t_to_dict(request)
        scripts = request_dict["scripts"]

        if len(scripts) == 0:
            return {
                "status": "success",
            }

        tasks_col = db["tasks"]

        def get_parent_task_id(script: dict[str, Any]) -> str:
            return str(script.get("parentTaskId") or "").strip()

        def is_main_script(script: dict[str, Any]) -> bool:
            return get_parent_task_id(script) == ""

        def get_main_task_id(script: dict[str, Any]) -> str:
            return script["taskId"] if is_main_script(script) else get_parent_task_id(script)

        main_task_doc_map: dict[str, dict[str, Any]] = {}
        for script in scripts:
            main_task_id = get_main_task_id(script)
            if main_task_id in main_task_doc_map:
                continue
            if not ObjectId.is_valid(main_task_id):
                continue
            main_task_doc = await tasks_col.find_one({"_id": ObjectId(main_task_id)})
            if main_task_doc:
                main_task_doc_map[main_task_id] = main_task_doc

        def is_event_sub_script(script: dict[str, Any]) -> bool:
            if is_main_script(script):
                return False
            first_step = (script.get("steps") or [{}])[0] or {}
            thread_type = (first_step.get("args") or {}).get("thread_type")
            return thread_type != "GENERAL"

        # parent(main task id) -> event sub tree list
        event_sub_trees_by_parent: dict[str, list[Step]] = defaultdict(list)
        for script in scripts:
            if not is_event_sub_script(script):
                continue

            parent_task_id = get_parent_task_id(script)
            if not parent_task_id:
                continue

            event_normal_steps = [
                s for s in script["steps"] if s.get("method", None) != "ActionPost"
            ]
            for event_step in event_normal_steps:
                event_sub_trees_by_parent[parent_task_id].append(
                    self.parse_step_context(event_step, depth=8)
                )

        tree_list = []
        for script in scripts:
            task_id = script["taskId"]
            if is_event_sub_script(script):
                continue

            repeat_count = script["repeatCount"]
            steps_tree = script["steps"]
            parent_task_id = script.get("parentTaskId", None)
            step_mode = script.get("stepMode", False)

            normal_steps_tree = []
            post_steps_tree = []

            for step in steps_tree:
                if step.get("method", None) == "ActionPost":
                    post_steps_tree.append(step)
                else:
                    normal_steps_tree.append(step)

            if len(post_steps_tree) > 1:
                raise HTTPException(status_code=400, detail=f"ActionPost step must be only one: {task_id}")

            main_task_id = get_main_task_id(script)
            robot_model = script.get("robotModel") or main_task_doc_map.get(main_task_id, {}).get("robotModel")
            if not robot_model:
                robot_model = self.script_executor.get_state(main_task_id).get("robot_model", None)
            category = self._robot_models.get(robot_model or "", {}).get("be_service", None)
            begin = script.get("begin", None)

            tree = self.build_tree_from_client(normal_steps_tree, task_id, robot_model, category, begin)
            post_tree = self.parse_step_context(post_steps_tree[0], depth=8) if len(post_steps_tree) > 0 else None

            tree_list.append(
                {
                    "taskId": task_id,
                    "tree": tree,
                    "post_tree": post_tree,
                    "repeat_count": repeat_count,
                    "robot_model": robot_model,
                    "category": category,
                    "step_mode": step_mode,
                    "parent_process_id": parent_task_id,
                    "event_sub_tree_list": event_sub_trees_by_parent.get(task_id, []),
                }
            )

        make_process_args_list = []
        for tree in tree_list:
            make_process_args_list.append(
                MakeProcessArgs(
                    process_id=tree["taskId"],
                    step=tree["tree"],
                    repeat_count=tree["repeat_count"],
                    robot_model=tree["robot_model"],
                    category=tree["category"],
                    step_mode=tree["step_mode"],
                    min_step_interval=0.5 if tree["step_mode"] else None,
                    is_ui_execution=True,
                    post_tree=tree["post_tree"],
                    parent_process_id=tree["parent_process_id"],
                    event_sub_tree_list=tree["event_sub_tree_list"],
                )
            )

        # stepMode 진행 중에는 기존 실행 프로세스를 재시작하지 않고 resume만 수행한다.
        step_mode_task_ids = [
            tree["taskId"] for tree in tree_list if tree.get("step_mode", False)
        ]
        if step_mode_task_ids and self._step_mode:
            alive_states = {
                task_id: self.script_executor.get_state(task_id) for task_id in step_mode_task_ids
            }
            alive_task_ids = [
                task_id for task_id, state in alive_states.items() if state.get("is_alive", False)
            ]
            if alive_task_ids:
                for task_id in alive_task_ids:
                    state = alive_states[task_id]
                    if state.get("state") in (
                        RB_Flow_Manager_ProgramState.PAUSED,
                        RB_Flow_Manager_ProgramState.WAITING,
                    ):
                        self.script_executor.resume(task_id)

                # stepMode 진행 요청에서는 종료된 태스크를 다시 시작하지 않는다.
                self._step_mode = True
                return {
                    "status": "success",
                }

            # reset/종료 이후 alive가 없으면 신규 start 경로로 내려간다.
            self._step_mode = False

        self.script_executor.start(make_process_args_list)

        # 모든 스크립트의 step_mode를 확인 (하나라도 True면 True)
        self._step_mode = any(tree["step_mode"] for tree in tree_list)

        return {
            "status": "success",
        }

    async def preview_reset_program(self):
        if self.script_executor:
            self.script_executor.reset()
        self._step_mode = False

        return {
            "status": "success",
        }

    async def preview_stop_program(self, request: Request_Preview_Stop_ProgramPD):
        request_dict = t_to_dict(request)
        task_ids = request_dict["taskIds"]

        for task_id in task_ids:
            self.script_executor.stop(task_id)

        self._step_mode = False

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

        for task_doc in tasks_docs:
            script_name = task_doc["scriptName"]
            script_path = task_doc["scriptPath"]
            extension = task_doc["extension"]
            robot_model = task_doc["robotModel"]
            category = self._robot_models[robot_model].get("be_service", None)

            tree = self._load_tree_from_script(f"{script_path}/{script_name}.{extension}")
            event_sub_tree_list = self._load_event_sub_trees_from_script(
                f"{script_path}/{script_name}.{extension}"
            )

            self.script_executor.start(
                MakeProcessArgs(
                    process_id=script_name,
                    step=tree,
                    repeat_count=repeat_count,
                    robot_model=robot_model,
                    category=category,
                    event_sub_tree_list=event_sub_tree_list,
                )
            )

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

    def program_dialog(self, request: Request_Program_Dialog):
        fire_and_log(socket_client.emit("rrs/program/dialog", request))

    def program_log(self, request: Request_Program_Log):
        content = f"[{request['robot_model']}] {request['content']}"

        log_level = request.get("level", "DEBUG")

        if log_level == "INFO":
            rb_log.info(content, disable_db=False)
        elif log_level == "WARNING":
            rb_log.warning(content, disable_db=False)
        elif log_level == "ERROR":
            rb_log.error(content, disable_db=False)
        elif log_level == "USER":
            rb_log.user(content, disable_db=False)
        elif log_level == "DEBUG":
            rb_log.debug(content, disable_db=False)
        elif log_level == "GENERAL":
            rb_log.general(content, disable_db=False)

    async def at_program_start(self, task_id: str, db: MongoDB):
        """
        Program 시작 시 실행하는 함수.
        """

        tasks_col = db["tasks"]
        task_doc = await tasks_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=404, detail=f"Task not found: {task_id}")

        robot_model = task_doc["robotModel"]
        be_service = self._robot_models[robot_model].get("be_service", None)

        if be_service == "manipulate":
            rb_manipulate_sdk.program.call_program_before(robot_model=robot_model, option=0)

    async def at_program_end(self, task_id: str, db: MongoDB):
        """
        Program 종료 시 실행하는 함수.
        """

        tasks_col = db["tasks"]
        task_doc = await tasks_col.find_one({"_id": ObjectId(task_id)})

        if not task_doc:
            raise HTTPException(status_code=404, detail=f"Task not found: {task_id}")

        robot_model = task_doc["robotModel"]
        be_service = self._robot_models[robot_model].get("be_service", None)

        if be_service == "manipulate":
            rb_manipulate_sdk.program.call_program_after(robot_model=robot_model, option=0)
