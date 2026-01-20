"""
[Email API]
"""



from fastapi import APIRouter
from rb_modules.log import RBLog

from app.features.network.application.network_service import (
    NetworkService,  # pylint: disable=import-error,no-name-in-module
)

network_router = APIRouter(
    tags=["Network"],
    prefix="/network"
)
network_service = NetworkService()
rb_log = RBLog()

@network_router.get("")
async def get_network():
    return await network_service.get_network()
