from typing import Annotated

from fastapi import (
    APIRouter,
    Query,
)
from rb_schemas.base import (
    Response_ReturnValuePD,
)

from .config_module import (
    ConfigService,
)
from .config_schema import (
    Request_Change_SpeedbarPD,
    Response_SpeedBarPD,
)

config_router = APIRouter(tags=["Config"])

config_service = ConfigService()


@config_router.get("/speedbar", response_model=Response_SpeedBarPD)
async def get_speedbar(
    components: Annotated[list[str], Query(alias="components[]")],
):
    """
    Get all speedbar
    """
    res = await config_service.get_all_speedbar(components=components)

    return res


@config_router.post("/speedbar", response_model=Response_ReturnValuePD)
async def change_speedbar(request: Request_Change_SpeedbarPD):
    """
    Change speedbar
    """
    return await config_service.control_speed_bar(**request.model_dump())
