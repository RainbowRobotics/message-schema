from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)

from .program_module import (
    ProgramService,
)

program_socket_router = RbSocketIORouter()
program_service = ProgramService()


@program_socket_router.on("{robot_model}/call_resume")
async def on_call_resume(data, robot_model: str):
    return to_json(await program_service.call_resume(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_pause")
async def on_call_pause(data, robot_model: str):
    return to_json(await program_service.call_pause(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_halt")
async def on_call_halt(data, robot_model: str):
    return to_json(await program_service.call_halt(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_program_before")
async def on_call_program_before(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_program_before(robot_model=robot_model, request=dict_data))

@program_socket_router.on("{robot_model}/call_program_after")
async def on_call_program_after(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_program_after(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_speedbar")
async def on_call_speedbar(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_speedbar(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_smoothjog_j")
async def on_call_smoothjog_j(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_smoothjog_j(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_smoothjog_l")
async def on_call_smoothjog_l(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_smoothjog_l(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_smoothjog_stop")
async def on_call_smoothjog_stop(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_smoothjog_stop(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_tickjog_j")
async def on_call_tickjog_j(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_tickjog_j(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_tickjog_l")
async def on_call_tickjog_l(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_tickjog_l(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_j")
async def on_call_move_j(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_j(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_l")
async def on_call_move_l(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_l(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_jb_clr")
async def on_call_move_jb_clr(robot_model: str):
    return to_json(await program_service.call_move_jb_clr(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_move_jb_add")
async def on_call_move_jb_add(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_jb_add(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_jb_run")
async def on_call_move_jb_run(robot_model: str):
    return to_json(await program_service.call_move_jb_run(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_move_lb_clr")
async def on_call_move_lb_clr(robot_model: str):
    return to_json(await program_service.call_move_lb_clr(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_move_lb_add")
async def on_call_move_lb_add(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_lb_add(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_lb_run")
async def on_call_move_lb_run(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_lb_run(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_xb_clr")
async def on_call_move_xb_clr(robot_model: str):
    return to_json(await program_service.call_move_xb_clr(robot_model=robot_model))


@program_socket_router.on("{robot_model}/call_move_xb_add")
async def on_call_move_xb_add(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_xb_add(robot_model=robot_model, request=dict_data))


@program_socket_router.on("{robot_model}/call_move_xb_run")
async def on_call_move_xb_run(data, robot_model: str):
    dict_data = t_to_dict(data)
    return to_json(await program_service.call_move_xb_run(robot_model=robot_model, request=dict_data))
