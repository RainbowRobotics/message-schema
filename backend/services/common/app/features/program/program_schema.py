# mypy: disable-error-code=misc
from __future__ import (
    annotations,
)

from datetime import (
    UTC,
    datetime,
)
from enum import Enum
from typing import Any, Literal, TypedDict

from pydantic import BaseModel, ConfigDict, Field, field_validator
from rb_database import PyObjectId
from rb_flow_manager.schema import RB_Flow_Manager_ProgramState
from rb_schemas.utility import Omit, Pick


class Request_Load_ProgramPD(BaseModel):
    programId: str


class Step_Base(BaseModel):
    stepId: PyObjectId = Field(alias="_id")
    parentStepId: str | None = Field(default=None)
    taskId: str
    programId: str
    groupId: str | None = Field(default=None)
    syncStepIds: list[str] | None = Field(default=None)
    targetStepId: str | None = Field(default=None)
    name: str
    variable: dict[str, Any] | None = Field(default=None)
    method: str
    args: dict[str, Any] | None = Field(default=None)
    func: str | None = Field(default=None)
    funcName: str | None = Field(default=None)
    summary: str | None = Field(default=None)
    doneScript: str | None = Field(default=None)
    order: int
    color: str | None = Field(default=None)
    highlight: bool = Field(default=False)
    disabled: bool = Field(default=False)
    memo: str | None = Field(default=None)
    allowChildren: list[str] | None = Field(default=None)
    allowParents: list[str] | None = Field(default=None)
    fixChildren: bool = Field(default=False)
    requireChildren: bool = Field(default=False)
    requireParent: bool = Field(default=False)
    disableDelete: bool = Field(default=False)
    groupLastItem: bool = Field(default=False)
    groupFirstItem: bool = Field(default=False)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.STOPPED)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())

    model_config = {
        "populate_by_name": True  # Pydantic v2에서 모델을 생성할 때 alias 대신 실제 필드명으로도 값을 채울 수 있게 허용하는 설정
    }


class Step_Tree_Base(Omit(Step_Base, "_id")):
    stepId: PyObjectId | None = Field(alias="_id", default=None)
    children: list[Step_Tree_Base] | None = Field(default=None)


class Request_Create_StepPD(Omit(Step_Base, "_id", "createdAt", "updatedAt")):
    stepId: str

    @field_validator("stepId")
    def validate_object_id(cls, v: str | None) -> str | None:  # pylint: disable=no-self-argument
        if v is None or not isinstance(v, str):
            raise ValueError("stepId must be a string or None")

        return v


class Response_Get_StepListPD(BaseModel):
    steps: list[Step_Base]
    stepTree: list[Step_Tree_Base]


class Request_Update_StepStatePD(BaseModel):
    stepId: str
    taskId: str
    state: RB_Flow_Manager_ProgramState
    error: str | None = None


class Request_Create_Multiple_StepPD(BaseModel):
    steps: list[Request_Create_StepPD]


class Request_Delete_StepsPD(BaseModel):
    step_ids: list[str]


class Response_Delete_StepsPD(BaseModel):
    stepDeleted: int


class Response_Upsert_StepsPD(BaseModel):
    inserted_count: int
    matched_count: int
    upserted_count: int
    modified_count: int
    deleted_count: int
    upserted_ids: list[str]


class TaskType(str, Enum):
    MAIN = "MAIN"
    SUB = "SUB"


class TaskExtension(str, Enum):
    PYTHON = "py"


class MainTaskBegin(BaseModel):
    position: Any
    is_enable: bool
    speed_ratio: float | None = Field(default=None)


class Task_Base(BaseModel):
    taskId: PyObjectId
    programId: str
    parentTaskId: str | None = Field(default=None)
    robotModel: str
    type: TaskType = Field(default=TaskType.MAIN)
    scriptName: str | None = Field(default=None)
    scriptPath: str | None = Field(default=None)
    begin: MainTaskBegin | None = Field(default=None)
    extension: TaskExtension = Field(default=TaskExtension.PYTHON)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.IDLE)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat(), exclude=True)
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat(), exclude=True)

class Response_Get_TaskInfoPD(BaseModel):
    task: Task_Base

class Request_Create_TaskPD(Omit(Task_Base, "taskId")):
    pass


class Request_Update_TaskPD(Pick(Task_Base, "taskId")):
    name: str | None = Field(default=None)
    programId: str | None = Field(default=None)
    robotModel: str | None = Field(default=None)
    type: TaskType | None = Field(default=None)
    scriptName: str | None = Field(default=None)
    scriptPath: str | None = Field(default=None)
    begin: MainTaskBegin | None = Field(default=None)
    extension: TaskExtension | None = Field(default=None)


class Request_Create_Multiple_TaskPD(BaseModel):
    tasks: list[Request_Create_TaskPD]


class Request_Update_Multiple_TaskPD(BaseModel):
    tasks: list[Request_Update_TaskPD]

class Response_Update_Multiple_TaskPD(BaseModel):
    tasks: list[Task_Base]


class Request_Delete_TasksPD(BaseModel):
    task_ids: list[str]


class Response_Delete_TasksPD(BaseModel):
    taskDeleted: int


class Program_Base(BaseModel):
    programId: PyObjectId = Field(alias="_id")
    name: str | None = Field(default=None)
    repeatCnt: int = Field(default=1)
    state: RB_Flow_Manager_ProgramState = Field(default=RB_Flow_Manager_ProgramState.STOPPED)
    createdAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())
    updatedAt: str = Field(default_factory=lambda: datetime.now(UTC).isoformat())


class Request_Get_Program_ListPD(BaseModel):
    state: RB_Flow_Manager_ProgramState | None = None
    search_name: str | None = None


class Response_Get_ProgramPD(BaseModel):
    program: Program_Base | None
    mainTasks: list[Task_Base] | None
    allSteps: dict[str, list[Step_Tree_Base]] | None


class Response_Get_Program_ListPD(BaseModel):
    programs: list[Program_Base]


class Create_Program_With_TaskPD(
    Omit(
        Task_Base,
        "_id",
        "taskId",
        "programId",
        "scriptName",
        "scriptPath",
        "state",
        "createdAt",
        "updatedAt",
    )
):
    model_config = ConfigDict(extra="allow")  # 기재되지 않은 필드도 허용하도록 설정


class Update_Program_With_TaskPD(
    Omit(Task_Base, "_id", "programId", "scriptName", "state", "createdAt", "updatedAt")
):
    robotModel: str | None = Field(default=None)
    type: TaskType | None = Field(default=None)
    scriptPath: str | None = Field(default=None)
    extension: TaskExtension | None = Field(default=None)
    state: RB_Flow_Manager_ProgramState | None = Field(default=None)

    model_config = ConfigDict(extra="allow")


class Request_Create_ProgramPD(Omit(Program_Base, "programId", "state", "createdAt", "updatedAt")):
    pass


class Request_Update_ProgramPD(Omit(Program_Base, "state", "createdAt", "updatedAt")):
    pass


class Response_Create_Program_And_TasksPD(BaseModel):
    program: Program_Base
    mainTasks: list[Task_Base]
    allSteps: dict[str, list[Step_Tree_Base]]


class Response_Update_ProgramPD(BaseModel):
    program: Program_Base


class Request_Clone_ProgramPD(BaseModel):
    programId: str
    newName: str


class Response_Clone_ProgramPD(BaseModel):
    new_program_id: str
    tasks_id_map: dict[str, str]


class Response_Delete_Program_And_TasksPD(BaseModel):
    programDeleted: int
    flowDeleted: int
    taskDeleted: int


class Request_Program_ExecutionPD(BaseModel):
    programId: str


class Request_Tasks_ExecutionPD(BaseModel):
    taskIds: list[str]
    repeatCount: int | None = Field(default=None)


class Response_Get_Task_ListPD(BaseModel):
    tasks: list[Task_Base]


class Response_Script_ExecutionPD(BaseModel):
    status: Literal["success", "error"]


class Response_Get_Script_ContextPD(BaseModel):
    context: str


class Client_StepPD(Omit(Step_Tree_Base, "order", "createdAt", "updatedAt")):
    stepId: str
    children: list[Client_StepPD] | None = Field(default=None)


class Client_Script_InfoPD(BaseModel):
    taskId: str
    repeatCount: int
    steps: list[Client_StepPD]
    stepMode: bool = Field(default=False)


class Request_Preview_Start_ProgramPD(BaseModel):
    scripts: list[Client_Script_InfoPD]

    model_config = ConfigDict(extra="ignore")


class Request_Preview_Stop_ProgramPD(BaseModel):
    taskIds: list[str]


class Request_Get_Script_ContextPD(BaseModel):
    taskId: str
    steps: list[Client_StepPD]


class PlayState(str, Enum):
    PLAY = "play"
    PAUSE = "pause"
    STOP = "stop"
    IDLE = "idle"


class Response_Get_Current_Program_StatePD(BaseModel):
    playState: PlayState
    taskPlayState: dict[str, PlayState]
    stepMode: bool

class Request_Program_Dialog(TypedDict):
    task_id: str
    title: str
    content: str

class Request_Program_Log(TypedDict):
    content: str
    robot_model: str
    level: Literal["INFO", "WARNING", "ERROR", "USER", "DEBUG", "GENERAL"]
