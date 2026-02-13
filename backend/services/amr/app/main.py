"""
[AMR Service]
"""
from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.control.control_api import amr_control_router
from app.features.map.map_api import amr_map_router
from app.features.map.map_zenoh import map_zenoh_router
from app.features.move.move_api import amr_move_router
from app.features.move.move_socket import move_socket_router
from app.features.move.move_zenoh import move_zenoh_router
from app.features.status.status_api import amr_status_router
from app.features.status.status_zenoh import status_zenoh_router

from .socket.socket_client import (
    socket_client,
)

setting = AppSettings()

app = create_app(
    settings=setting,
    socket_client=socket_client,
    zenoh_routers=[status_zenoh_router, move_zenoh_router, map_zenoh_router],
    api_routers=[
        amr_move_router,
        amr_control_router,
        amr_map_router,
        amr_status_router,
    ],
    socket_routers=[move_socket_router],
    # bg_tasks=[]
    # bg_tasks=[move_mongo_service.moveDBinit],  # lifespan에서 실행됨
)
