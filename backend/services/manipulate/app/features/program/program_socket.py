from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    to_json,
)

from .program_module import (
    ProgramService,
)
from .program_schema import (
    Request_MotionHaltPD,
    Request_MotionSpeedBarPD,
    Request_MoveJBAddPD,
    Request_MoveJBClrPD,
    Request_MoveJBRunPD,
    Request_MoveJPD,
    Request_MoveLBAddPD,
    Request_MoveLBClrPD,
    Request_MoveLBRunPD,
    Request_MoveLPD,
    Request_MoveSmoothJogJPD,
    Request_MoveSmoothJogLPD,
    Request_MoveSmoothJogStopPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
    Request_MoveXBAddPD,
    Request_MoveXBClrPD,
    Request_MoveXBRunPD,
    Request_PausePD,
    Request_ProgramAfterPD,
    Request_ProgramBeforePD,
    Request_ResumePD,
)

program_socket_router = RbSocketIORouter()
program_service = ProgramService()


# ==============================================================================
# 1. System Control
# ==============================================================================

@program_socket_router.on("{robot_model}/call_resume")
async def on_call_resume(data: Request_ResumePD, robot_model: str):
    return to_json(await program_service.call_resume(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_pause")
async def on_call_pause(data: Request_PausePD, robot_model: str):
    return to_json(await program_service.call_pause(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_halt")
async def on_call_halt(data: Request_MotionHaltPD, robot_model: str):
    return to_json(await program_service.call_halt(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_program_before")
async def on_call_program_before(data: Request_ProgramBeforePD, robot_model: str):
    return to_json(program_service.call_program_before(robot_model=robot_model, request=data))

@program_socket_router.on("{robot_model}/call_program_after")
async def on_call_program_after(data: Request_ProgramAfterPD, robot_model: str):
    return to_json(program_service.call_program_after(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_speedbar")
async def on_call_speedbar(data: Request_MotionSpeedBarPD, robot_model: str):
    return to_json(program_service.call_speedbar(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_smoothjog_j")
async def on_call_smoothjog_j(data: Request_MoveSmoothJogJPD, robot_model: str):
    return to_json(program_service.call_smoothjog_j(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_smoothjog_l")
async def on_call_smoothjog_l(data: Request_MoveSmoothJogLPD, robot_model: str):
    return to_json(program_service.call_smoothjog_l(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_smoothjog_stop")
async def on_call_smoothjog_stop(data: Request_MoveSmoothJogStopPD, robot_model: str):
    return to_json(program_service.call_smoothjog_stop(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_tickjog_j")
async def on_call_tickjog_j(data: Request_MoveTickJogJPD, robot_model: str):
    return to_json(program_service.call_tickjog_j(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_tickjog_l")
async def on_call_tickjog_l(data: Request_MoveTickJogLPD, robot_model: str):
    return to_json(await program_service.call_tickjog_l(robot_model=robot_model, request=data))


# ==============================================================================
# 3. Direct Move Control
# ==============================================================================

@program_socket_router.on("{robot_model}/call_move_j")
async def on_call_move_j(data: Request_MoveJPD, robot_model: str):
    return to_json(program_service.call_move_j(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_l")
async def on_call_move_l(data: Request_MoveLPD, robot_model: str):
    return to_json(program_service.call_move_l(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_jb_clr")
async def on_call_move_jb_clr(data: Request_MoveJBClrPD, robot_model: str):
    return to_json(await program_service.call_move_jb_clr(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_jb_add")
async def on_call_move_jb_add(data: Request_MoveJBAddPD, robot_model: str):
    return to_json(await program_service.call_move_jb_add(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_jb_run")
async def on_call_move_jb_run(data: Request_MoveJBRunPD, robot_model: str):
    return to_json(await program_service.call_move_jb_run(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_lb_clr")
async def on_call_move_lb_clr(data: Request_MoveLBClrPD, robot_model: str):
    return to_json(await program_service.call_move_lb_clr(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_lb_add")
async def on_call_move_lb_add(data: Request_MoveLBAddPD, robot_model: str):
    return to_json(await program_service.call_move_lb_add(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_lb_run")
async def on_call_move_lb_run(data: Request_MoveLBRunPD, robot_model: str):
    return to_json(await program_service.call_move_lb_run(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_xb_clr")
async def on_call_move_xb_clr(data: Request_MoveXBClrPD, robot_model: str):
    return to_json(await program_service.call_move_xb_clr(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_xb_add")
async def on_call_move_xb_add(data: Request_MoveXBAddPD, robot_model: str):
    return to_json(await program_service.call_move_xb_add(robot_model=robot_model, request=data))


@program_socket_router.on("{robot_model}/call_move_xb_run")
async def on_call_move_xb_run(data: Request_MoveXBRunPD, robot_model: str):
    return to_json(await program_service.call_move_xb_run(robot_model=robot_model, request=data))
