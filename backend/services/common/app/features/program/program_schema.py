# mypy: disable-error-code=misc
from __future__ import annotations

from datetime import UTC, datetime
from enum import Enum
from typing import Any

from bson import ObjectId
from pydantic import BaseModel, ConfigDict, Field, field_validator
from rb_database import PyObjectId
from rb_flow_manager.control import RB_Flow_Manager_ProgramState
from rb_schemas.utility import Omit, Pick


class ProgramStatus(str, Enum):
    STOPPED = "STOPPED"
    RUNNING = "RUNNING"
    WAITING = "WAITING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    COMPLETED = "COMPLETED"


class Request_Set_Task_StatusPD(BaseModel):
    program_id: str
    sync_task_ids: list[str] | None = []
    node_path: list[str]
    offset: int
    status: ProgramStatus


class Response_Task_StatusPD(BaseModel):
    robotModel: str
    programId: str
    taskId: str
    syncTaskIds: list[str]
    nodePath: list[str]
    offset: int
    status: ProgramStatus
    createdAt: str
    updatedAt: str


class TaskType(str, Enum):
    NODE = "NODE"
    STEP = "STEP"


class Task_Base(BaseModel):
    taskId: PyObjectId = Field(alias="_id")
    nodeId: str | None = Field(default=None)
    flowId: str
    syncTaskIds: list[str] | None = Field(default=None)
    type: TaskType
    name: str
    method: str
    args: list[Any] | None = Field(default=None)
    contents: str | None = Field(default=None)
    summary: str | None = Field(default=None)
    order: int
    color: str | None = Field(default=None)
    highlight: bool = Field(default=False)
    disabled: bool = Field(default=False)
    memo: str | None = Field(default=None)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.STOPPED)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())

    model_config = {
        "populate_by_name": True  # Pydantic v2에서 모델을 생성할 때 alias 대신 실제 필드명으로도 값을 채울 수 있게 허용하는 설정
    }


class Task_Tree_Base(Task_Base):
    steps: list[Task_Tree_Base] | None = Field(default=None)


class Request_Create_TaskPD(Omit(Task_Base, "_id", "taskId", "createdAt", "updatedAt")):
    taskId: str | None = Field(
        default=None,
    )

    @field_validator("taskId")
    def validate_object_id(cls, v: str | None) -> str | None:  # pylint: disable=no-self-argument
        if v is None:
            return v
        if not ObjectId.is_valid(v):
            raise ValueError(
                "is not a valid ObjectId, it must be a 12-byte input or a 24-character hex string"
            )
        return v


class Response_Get_TaskListPD(BaseModel):
    tasks: list[Task_Base]
    taskTree: list[Task_Tree_Base]


class Request_Create_Multiple_TaskPD(BaseModel):
    tasks: list[Request_Create_TaskPD]


class Request_Delete_TasksPD(BaseModel):
    task_ids: list[str]


class Response_Delete_TasksPD(BaseModel):
    taskDeleted: int


class FlowType(str, Enum):
    MAIN = "MAIN"
    SUB = "SUB"


class FlowExtension(str, Enum):
    PYTHON = "py"


class Flow_Base(BaseModel):
    flowId: PyObjectId
    name: str
    programId: str
    robotModel: str
    type: FlowType = Field(default=FlowType.MAIN)
    scriptName: str
    scriptPath: str
    extendsion: FlowExtension = Field(default=FlowExtension.PYTHON)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.STOPPED)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat(), exclude=True)
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat(), exclude=True)


class Request_Create_FlowPD(Omit(Flow_Base, "flowId")):
    pass


class Request_Update_FlowPD(Pick(Flow_Base, "flowId")):
    name: str | None = Field(default=None)
    programId: str | None = Field(default=None)
    robotModel: str | None = Field(default=None)
    type: FlowType | None = Field(default=None)
    scriptName: str | None = Field(default=None)
    scriptPath: str | None = Field(default=None)
    extendsion: FlowExtension | None = Field(default=None)
    state: RB_Flow_Manager_ProgramState | None = Field(default=None)


class Request_Create_Multiple_FlowPD(BaseModel):
    flows: list[Request_Create_FlowPD]


class Request_Update_Multiple_FlowPD(BaseModel):
    flows: list[Request_Update_FlowPD]


class Request_Delete_FlowsPD(BaseModel):
    flow_ids: list[str]


class Response_Delete_FlowsPD(BaseModel):
    flowDeleted: int
    taskDeleted: int


class Program_Base(BaseModel):
    programId: PyObjectId
    name: str
    repeatCnt: int = Field(default=1)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.STOPPED)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())


class Request_Get_Program_ListPD(BaseModel):
    state: RB_Flow_Manager_ProgramState | None = None
    search_name: str | None = None


class Response_Get_ProgramPD(BaseModel):
    program: Program_Base
    flows: list[Flow_Base]


class Create_Program_With_FlowPD(
    Omit(Flow_Base, "_id", "flowId", "programId", "name", "scriptName", "createdAt", "updatedAt")
):
    model_config = ConfigDict(extra="allow")  # 기재되지 않은 필드도 허용하도록 설정


class Update_Program_With_FlowPD(
    Omit(Flow_Base, "_id", "programId", "name", "scriptName", "createdAt", "updatedAt")
):
    robotModel: str | None = Field(default=None)
    type: FlowType | None = Field(default=None)
    scriptPath: str | None = Field(default=None)
    extendsion: FlowExtension | None = Field(default=None)
    state: RB_Flow_Manager_ProgramState | None = Field(default=None)

    model_config = ConfigDict(extra="allow")


class Update_ProgramPD(Omit(Program_Base, "createdAt", "updatedAt")):
    pass


class Create_ProgramPD(Omit(Program_Base, "_id", "programId", "createdAt", "updatedAt")):
    name: str | None = Field(default=None)
    repeatCnt: int | None = Field(default=None)


class Request_Create_ProgramPD(BaseModel):
    program: Create_ProgramPD
    flows: list[Create_Program_With_FlowPD] = Field(default_factory=list)


class Request_Update_ProgramPD(BaseModel):
    program: Update_ProgramPD
    flows: list[Update_Program_With_FlowPD] = Field(default_factory=list)


class Response_Upsert_Program_And_FlowsPD(BaseModel):
    program: Program_Base
    flows: list[Flow_Base]


class Response_Delete_Program_And_FlowsPD(BaseModel):
    programDeleted: int
    flowDeleted: int
    taskDeleted: int
