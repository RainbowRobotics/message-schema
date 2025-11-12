from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
)

from .program_module import (
    ProgramService,
)
from .program_schema import (
    PauseRequestPD,
    Request_MoveJBAddPD,
    Request_MoveJPD,
    Request_MoveLBAddPD,
    Request_MoveLBRunPD,
    Request_MoveLPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
    ResumeRequestPD,
    SmoothJogJRequestPD,
    SmoothJogLRequestPD,
    SmoothJogStopRequestPD,
    SpeedBarRequestPD,
)

program_service = ProgramService()

program_router = APIRouter(tags=["Program"])


@program_router.post("/{robot_model}/call_resume", response_model=Response_ReturnValuePD)
async def resume(robot_model: str, request: ResumeRequestPD):
    res = await program_service.call_resume(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_pause", response_model=Response_ReturnValuePD)
async def pause(robot_model: str, request: PauseRequestPD):
    res = await program_service.call_pause(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_speedbar", response_model=Response_ReturnValuePD)
async def speedbar(robot_model: str, request: SpeedBarRequestPD):
    res = await program_service.call_speedbar(robot_model=robot_model, speedbar=request.speedbar)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_j", response_model=Response_ReturnValuePD)
async def smoothjog_j(robot_model: str, request: SmoothJogJRequestPD):
    res = await program_service.call_smoothjog_j(
        robot_model=robot_model,
        targetspeed=request.targetspeed,
        frame=request.frame,
        unit=request.unit,
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_l", response_model=Response_ReturnValuePD)
async def smoothjog_l(robot_model: str, request: SmoothJogLRequestPD):
    res = await program_service.call_smoothjog_l(
        robot_model=robot_model,
        targetspeed=request.targetspeed,
        frame=request.frame,
        unit=request.unit,
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_stop", response_model=Response_ReturnValuePD)
async def smoothjog_stop(robot_model: str, request: SmoothJogStopRequestPD):
    res = await program_service.call_smoothjog_stop(
        robot_model=robot_model, stoptime=request.stoptime
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_j", response_model=Response_ReturnValuePD)
async def move_j(robot_model: str, request: Request_MoveJPD):
    res = await program_service.call_move_j(
        robot_model=robot_model, target=request.target, speed=request.speed
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_l", response_model=Response_ReturnValuePD)
async def move_l(robot_model: str, request: Request_MoveLPD):
    res = await program_service.call_move_l(
        robot_model=robot_model, target=request.target, speed=request.speed
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_tickjog_j", response_model=Response_ReturnValuePD)
async def tickjog_j(robot_model: str, request: Request_MoveTickJogJPD):
    res = program_service.call_tickjog_j(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_tickjog_l", response_model=Response_ReturnValuePD)
async def tickjog_l(robot_model: str, request: Request_MoveTickJogLPD):
    res = program_service.call_tickjog_l(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_jb_clr", response_model=Response_ReturnValuePD)
async def move_jb_clr(robot_model: str):
    res = program_service.call_move_jb_clr(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_jb_add", response_model=Response_ReturnValuePD)
async def move_jb_add(robot_model: str, request: Request_MoveJBAddPD):
    res = program_service.call_move_jb_add(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_jb_run", response_model=Response_ReturnValuePD)
async def move_jb_run(robot_model: str):
    res = program_service.call_move_jb_run(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_clr", response_model=Response_ReturnValuePD)
async def move_lb_clr(robot_model: str):
    res = program_service.call_move_lb_clr(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_add", response_model=Response_ReturnValuePD)
async def move_lb_add(robot_model: str, request: Request_MoveLBAddPD):
    res = program_service.call_move_lb_add(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_run", response_model=Response_ReturnValuePD)
async def move_lb_run(robot_model: str, request: Request_MoveLBRunPD):
    res = program_service.call_move_lb_run(robot_model=robot_model, request=request)
    return JSONResponse(res)
