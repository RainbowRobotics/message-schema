from app.modules.whomai.whoami_module_service import WhoamiService
from fastapi import APIRouter
from fastapi.responses import JSONResponse

whoami_service = WhoamiService()

whoami_router = APIRouter()


@whoami_router.get("/whoami")
async def whoami():
    res = await whoami_service.get_whoami()

    return JSONResponse(res)
