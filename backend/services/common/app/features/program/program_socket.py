from rb_database import get_db
from rb_socketio import RbSocketIORouter
from rb_utils.parser import t_to_dict, to_json

from .program_module import ProgramService

program_socket_router = RbSocketIORouter()
program_service = ProgramService()


@program_socket_router.on("program/{program_id}")
async def on_change_speedbar(data, program_id: str):
    db = await get_db()
    res = await program_service.get_program_info(program_id=program_id, db=db)
    return to_json(res)


@program_socket_router.on("programs")
async def on_get_program_list(data):
    data_dict = t_to_dict(data)
    state = data_dict.get("state")
    search_name = data_dict.get("search_name")

    db = await get_db()

    res = await program_service.get_program_list(state=state, search_name=search_name, db=db)
    return to_json(res)


@program_socket_router.on("program/create")
async def on_create_program(data):
    data_dict = t_to_dict(data)

    db = await get_db()

    res = await program_service.create_program_and_flows(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/edit")
async def on_edit_program(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.update_program_and_flows(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/delete/{program_id}")
async def on_delete_program(data, program_id: str):
    db = await get_db()
    res = await program_service.delete_program(program_id=program_id, db=db)
    return to_json(res)


@program_socket_router.on("program/task/{task_id}")
async def on_get_task(data, task_id: str):
    db = await get_db()
    res = await program_service.get_task(task_id=task_id, db=db)
    return to_json(res)


@program_socket_router.on("program/tasks/{flow_id}")
async def on_get_task_list(data, flow_id: str):
    db = await get_db()
    res = await program_service.get_task_list(flow_id=flow_id, db=db)
    return to_json(res)


@program_socket_router.on("program/tasks/upsert")
async def on_upsert_tasks(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.upsert_tasks(request=data_dict, db=db)
    return to_json(res)


@program_socket_router.on("program/tasks/delete")
async def on_delete_tasks(data):
    data_dict = t_to_dict(data)

    db = await get_db()
    res = await program_service.delete_tasks(request=data_dict, db=db)
    return to_json(res)
