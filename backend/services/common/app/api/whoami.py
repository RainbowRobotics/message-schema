from fastapi import APIRouter
from fastapi.responses import JSONResponse

from app.modules.whomai.whoami_module_schema import Response_CallWhoamIPD
from app.modules.whomai.whoami_module_service import WhoamiService

whoami_service = WhoamiService()

whoami_router = APIRouter(tags=["Whoami"])


@whoami_router.get("/{robot_model}/call_whoami", response_model=Response_CallWhoamIPD)
async def whoami(robot_model: str):
    res = await whoami_service.get_whoami(robot_model)

    if res["err"]:
        return JSONResponse(status_code=400, content=res["err"])

    return JSONResponse(res["dict_payload"])
