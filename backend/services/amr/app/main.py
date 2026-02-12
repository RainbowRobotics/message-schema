"""
[AMR Service]
"""
from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.control.control_api import amr_control_router
from app.features.map.map_api import amr_map_router
from app.features.move.move_api import amr_move_router
from app.features.move.move_zenoh import move_zenoh_router
from app.socket.socket_client import (
    amr_zenoh_router,
)
from app.features.move.move_socket import move_socket_router

from .socket.socket_client import (
    socket_client,
)

setting = AppSettings()

# move_mongo_service = MoveMongoService()


app = create_app(
    settings=setting,
    socket_client=socket_client,
    zenoh_routers=[amr_zenoh_router, move_zenoh_router],
    api_routers=[
        amr_move_router,
        amr_control_router,
        amr_map_router,
        # amr_localization_router,
        # amr_map_router,
        # amr_network_router,
        # amr_setting_router,
        # amr_update_router
    ],
    socket_routers=[move_socket_router],
    # bg_tasks=[]
    # bg_tasks=[move_mongo_service.moveDBinit],  # lifespan에서 실행됨
)


# async def exception_handler(request: Request, exc: Exception):
#     if isinstance(exc, ServiceException):
#         rb_log.error(f"ServiceException: {exc.message}, {exc.status_code}")
#         return JSONResponse(status_code=exc.status_code, content={"error": exc.message})
#     else:
#         rb_log.error(f"Exception: {exc}")
#         return JSONResponse(status_code=500, content={"error": "Internal Server Error", "message": str(exc)})


# app.add_exception_handler(ServiceException, exception_handler)
