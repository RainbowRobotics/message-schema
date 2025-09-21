from app.api.api_schema import BaseControlResponsePD
from app.api.program.program_api_schema import (
    PauseRequestPD,
    ResumeRequestPD,
    SmoothJogJRequestPD,
    SmoothJogLRequestPD,
    SmoothJogStopRequestPD,
    SpeedBarRequestPD,
)
from app.modules.program.program_module_service import ProgramService
from fastapi import APIRouter
from fastapi.responses import JSONResponse

program_service = ProgramService()

program_router = APIRouter()


@program_router.post("/{robot_model}/call_resume", response_model=BaseControlResponsePD)
async def resume(robot_model: str, request: ResumeRequestPD):
    res = await program_service.call_resume(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_pause", response_model=BaseControlResponsePD)
async def pause(robot_model: str, request: PauseRequestPD):
    res = await program_service.call_pause(robot_model=robot_model)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_speedbar", response_model=BaseControlResponsePD)
async def speedbar(robot_model: str, request: SpeedBarRequestPD):
    res = await program_service.call_speedbar(robot_model=robot_model, speedbar=request.speedbar)
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_j", response_model=BaseControlResponsePD)
async def smoothjog_j(robot_model: str, request: SmoothJogJRequestPD):
    print("Received SmoothJogJRequestPD:", request)  # Debug print statement
    res = await program_service.call_smoothjogj(
        robot_model=robot_model,
        targetspeed=request.targetspeed,
        frame=request.frame,
        unit=request.unit,
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_l", response_model=BaseControlResponsePD)
async def smoothjog_l(robot_model: str, request: SmoothJogLRequestPD):
    res = await program_service.call_smoothjog_l(
        robot_model=robot_model,
        targetspeed=request.targetspeed,
        frame=request.frame,
        unit=request.unit,
    )
    return JSONResponse(res)


@program_router.post("/{robot_model}/call_smoothjog_stop", response_model=BaseControlResponsePD)
async def smoothjog_stop(robot_model: str, request: SmoothJogStopRequestPD):
    res = await program_service.call_smoothjog_stop(
        robot_model=robot_model, stoptime=request.stoptime
    )
    return JSONResponse(res)
