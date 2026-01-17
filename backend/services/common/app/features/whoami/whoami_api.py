from fastapi import APIRouter
from fastapi.responses import (
    JSONResponse,
)

from .whoami_module import (
    WhoamiService,
)
from .whoami_schema import (
    Response_CallWhoamIPD,
)

whoami_service = WhoamiService()

whoami_router = APIRouter(tags=["Whoami"])


@whoami_router.get("/{robot_model}/call_whoami", response_model=Response_CallWhoamIPD)
async def whoami(robot_model: str):
    res = await whoami_service.get_whoami(robot_model)

    return JSONResponse(res.get("dict_payload", {}))
