from rb_socketio import RbSocketIORouter
from utils.parser import t_to_dict, to_json

from app.modules.program.program_module_service import ProgramService

program_socket_router = RbSocketIORouter()
program_service = ProgramService()


@program_socket_router.on("{robot_model}/call_resume")
async def on_call_resume(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_resume", dict_data, robot_model, flush=True)

    res = await program_service.call_resume(robot_model=robot_model)

    return to_json(res)


@program_socket_router.on("{robot_model}/call_pause")
async def on_call_pause(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_pause", dict_data, robot_model, flush=True)

    res = await program_service.call_pause(robot_model=robot_model)

    return to_json(res)


@program_socket_router.on("{robot_model}/call_speedbar")
async def on_call_speedbar(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_speedbar", dict_data, robot_model, flush=True)

    res = await program_service.call_speedbar(
        robot_model=robot_model, speedbar=dict_data["speedbar"]
    )

    return to_json(res)


@program_socket_router.on("{robot_model}/call_smoothjog_j")
async def on_call_smoothjog_j(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_smoothjog_j", dict_data, robot_model, flush=True)

    res = await program_service.call_smoothjog_j(
        robot_model=robot_model,
        targetspeed=dict_data["targetspeed"],
        frame=dict_data["frame"],
        unit=dict_data["unit"],
    )

    return to_json(res)


@program_socket_router.on("{robot_model}/call_smoothjog_l")
async def on_call_smoothjog_l(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_smoothjog_l", dict_data, robot_model, flush=True)

    res = await program_service.call_smoothjog_l(
        robot_model=robot_model,
        targetspeed=dict_data["targetspeed"],
        frame=dict_data["frame"],
        unit=dict_data["unit"],
    )

    return to_json(res)


@program_socket_router.on("{robot_model}/call_smoothjog_stop")
async def on_call_smoothjog_stop(data, robot_model: str):
    dict_data = t_to_dict(data)

    print("call_smoothjog_stop", dict_data, robot_model, flush=True)

    res = await program_service.call_smoothjog_stop(
        robot_model=robot_model, stoptime=dict_data["stoptime"]
    )

    return to_json(res)
