from rb_database import get_db
from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)
from rb_zenoh.client import (
    ZenohClient,
)

from .program_module import (
    ProgramService,
)

program_socket_router = RbSocketIORouter()
zenoh_client = ZenohClient()
program_service = ProgramService()


@program_socket_router.on("program/{program_id}")
async def on_change_speedbar(data, program_id: str):
    db = await get_db()
    res = await program_service.get_program_info(program_id=program_id, db=db)
    return to_json(res)


@program_socket_router.on("programs")
async def on_get_program_list(data):
    data_dict = t_to_dict(data)
    search_name = data_dict.get("search_name")

    db = await get_db()

    res = await program_service.get_program_list(search_name=search_name, db=db)
    return to_json(res)


@program_socket_router.on("program/create")
async def on_create_program(data):
    data_dict = t_to_dict(data)

    db = await get_db()

    res = await program_service.create_program_and_tasks(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/edit")
async def on_edit_program(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.update_program(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/delete/{program_id}")
async def on_delete_program(data, program_id: str):
    db = await get_db()
    res = await program_service.delete_program(program_id=program_id, db=db)
    return to_json(res)


@program_socket_router.on("program/step/{step_id}")
async def on_get_step(data, step_id: str):
    db = await get_db()
    res = await program_service.get_step(step_id=step_id, db=db)
    return to_json(res)


@program_socket_router.on("program/steps/{task_id}")
async def on_get_step_list(data, task_id: str):
    db = await get_db()
    res = await program_service.get_step_list(task_id=task_id, db=db)
    return to_json(res)

@program_socket_router.on("program/task/list")
async def on_get_task_list(data):
    data_dict = t_to_dict(data)
    robot_model = data_dict.get("robot_model", None)
    task_type = data_dict.get("task_type", None)
    search_text = data_dict.get("search_text", None)
    db = await get_db()

    res = await program_service.get_task_list(robot_model=robot_model, task_type=task_type, search_text=search_text, db=db)
    return to_json(res)


@program_socket_router.on("program/tasks/upsert")
async def on_upsert_tasks(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.upsert_steps(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/tasks/delete")
async def on_delete_tasks(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.delete_tasks(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/pause")
async def on_pause_program(data):
    # data_dict = t_to_dict(data)

    return await program_service.call_resume_or_pause(
        is_pause=True,
        # program_id=data_dict.get("programId"),
        # flow_id=data_dict.get("flowId"),
    )


@program_socket_router.on("program/resume")
async def on_resume_program(data):
    # data_dict = t_to_dict(data)

    return await program_service.call_resume_or_pause(
        is_pause=False,
    )

@program_socket_router.on("program/{robot_model}/variables")
async def on_get_executor_variables(data, robot_model: str):
    res = program_service.get_executor_variables(robot_model=robot_model)
    return to_json(res)
