from fastapi import APIRouter
from fastapi.responses import JSONResponse

from app.modules.whomai.whoami_module_schema import Response_CallWhoamIPD
from app.modules.whomai.whoami_module_service import WhoamiService

whoami_service = WhoamiService()

whoami_router = APIRouter(tags=["Whoami"])


@whoami_router.get("/whoami", response_model=Response_CallWhoamIPD)
async def whoami():
    res = await whoami_service.get_whoami()

    return JSONResponse(res)
