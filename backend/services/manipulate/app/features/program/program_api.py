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
    Request_MotionHaltPD,
    Request_MotionSpeedBarPD,
    Request_Move_SmoothJogJPD,
    Request_Move_SmoothJogLPD,
    Request_Move_SmoothJogStopPD,
    Request_MoveJBAddPD,
    Request_MoveJPD,
    Request_MoveLBAddPD,
    Request_MoveLBRunPD,
    Request_MoveLPD,
    Request_MoveTickJogJPD,
    Request_MoveTickJogLPD,
    Request_MoveXBAddPD,
    Request_MoveXBRunPD,
    Request_PausePD,
    Request_ProgramAfterPD,
    Request_ProgramBeforePD,
    Request_RelativeMovePD,
    Request_ResumePD,
)

program_service = ProgramService()

program_router = APIRouter(tags=["Program"])


# ==============================================================================
# 1. System Control
# ==============================================================================

@program_router.post("/{robot_model}/call_resume", response_model=Response_ReturnValuePD)
async def call_resume(robot_model: str, request: Request_ResumePD):
    res = await program_service.call_resume(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_pause", response_model=Response_ReturnValuePD)
async def call_pause(robot_model: str, request: Request_PausePD):
    res = await program_service.call_pause(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_halt", response_model=Response_ReturnValuePD)
async def call_halt(robot_model: str, request: Request_MotionHaltPD):
    res = await program_service.call_halt(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_program_before", response_model=Response_ReturnValuePD)
async def call_program_before(robot_model: str, request: Request_ProgramBeforePD):
    return JSONResponse(await program_service.call_program_before(robot_model=robot_model, request=request))


@program_router.post("/{robot_model}/call_program_after", response_model=Response_ReturnValuePD)
async def call_program_after(robot_model: str, request: Request_ProgramAfterPD):
    return JSONResponse(await program_service.call_program_after(robot_model=robot_model, request=request))


# ==============================================================================
# 2. Jog Control
# ==============================================================================

@program_router.post("/{robot_model}/call_speedbar", response_model=Response_ReturnValuePD)
async def call_speedbar(robot_model: str, request: Request_MotionSpeedBarPD):
    return JSONResponse(await program_service.call_speedbar(robot_model=robot_model, request=request))


@program_router.post("/{robot_model}/call_smoothjog_j", response_model=Response_ReturnValuePD)
async def call_smoothjog_j(robot_model: str, request: Request_Move_SmoothJogJPD):
    res = await program_service.call_smoothjog_j(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_l", response_model=Response_ReturnValuePD)
async def call_smoothjog_l(robot_model: str, request: Request_Move_SmoothJogLPD):
    res = await program_service.call_smoothjog_l(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_stop", response_model=Response_ReturnValuePD)
async def call_smoothjog_stop(robot_model: str, request: Request_Move_SmoothJogStopPD):
    res = await program_service.call_smoothjog_stop(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_tickjog_j", response_model=Response_ReturnValuePD)
async def call_tickjog_j(robot_model: str, request: Request_MoveTickJogJPD):
    res = await program_service.call_tickjog_j(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_tickjog_l", response_model=Response_ReturnValuePD)
async def call_tickjog_l(robot_model: str, request: Request_MoveTickJogLPD):
    res = await program_service.call_tickjog_l(robot_model=robot_model, request=request)
    return JSONResponse(res)

# ==============================================================================
# 3. Direct Move Control
# ==============================================================================

@program_router.post("/{robot_model}/call_move_j", response_model=Response_ReturnValuePD)
async def call_move_j(robot_model: str, request: Request_MoveJPD):
    res = await program_service.call_move_j(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_l", response_model=Response_ReturnValuePD)
async def call_move_l(robot_model: str, request: Request_MoveLPD):
    res = await program_service.call_move_l(robot_model=robot_model, request=request)
    return JSONResponse(res)

# ==============================================================================
# 4. Blend Move Control (JB, LB, XB)
# ==============================================================================

@program_router.post("/{robot_model}/call_move_jb_clr", response_model=Response_ReturnValuePD)
async def call_move_jb_clr(robot_model: str):
    res = await program_service.call_move_jb_clr(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_jb_add", response_model=Response_ReturnValuePD)
async def call_move_jb_add(robot_model: str, request: Request_MoveJBAddPD):
    res = await program_service.call_move_jb_add(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_jb_run", response_model=Response_ReturnValuePD)
async def call_move_jb_run(robot_model: str):
    res = await program_service.call_move_jb_run(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_clr", response_model=Response_ReturnValuePD)
async def call_move_lb_clr(robot_model: str):
    res = await program_service.call_move_lb_clr(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_add", response_model=Response_ReturnValuePD)
async def call_move_lb_add(robot_model: str, request: Request_MoveLBAddPD):
    res = await program_service.call_move_lb_add(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_lb_run", response_model=Response_ReturnValuePD)
async def call_move_lb_run(robot_model: str, request: Request_MoveLBRunPD):
    res = await program_service.call_move_lb_run(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_xb_clr", response_model=Response_ReturnValuePD)
async def call_move_xb_clr(robot_model: str):
    res = await program_service.call_move_xb_clr(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_xb_add", response_model=Response_ReturnValuePD)
async def call_move_xb_add(robot_model: str, request: Request_MoveXBAddPD):
    res = await program_service.call_move_xb_add(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_move_xb_run", response_model=Response_ReturnValuePD)
async def call_move_xb_run(robot_model: str, request: Request_MoveXBRunPD):
    res = await program_service.call_move_xb_run(robot_model=robot_model, request=request)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_relative_move", response_model=Response_ReturnValuePD)
async def call_relative_move(robot_model: str, request: Request_RelativeMovePD):
    res = await program_service.call_relative_move(robot_model=robot_model, request=request)
    return JSONResponse(res)
