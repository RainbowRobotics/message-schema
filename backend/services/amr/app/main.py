"""
[AMR Service]
"""
from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from .features.gateway.amr_api_control import (
    amr_control_router,
)
from .features.gateway.amr_api_localization import (
    amr_localization_router,
)
from .features.gateway.amr_api_map import (
    amr_map_router,
)
from .features.gateway.amr_api_network import (
    amr_network_router,
)
from .features.gateway.amr_api_setting import (
    amr_setting_router,
)
from .features.gateway.amr_api_update import (
    amr_update_router,
)
from .features.gateway.amr_zenoh import (
    amr_zenoh_router,
)
from .features.move.src.adapters.input.amr_move_api import (
    amr_move_router,
)
from .features.move.src.adapters.input.amr_move_zenoh import (
    move_zenoh_router,
)
from .features.move.src.application.amr_move_service import (
    AmrMoveService,
)
from .socket.socket_client import (
    socket_client,
)

setting = AppSettings()

move_service = AmrMoveService()
# move_mongo_service = MoveMongoService()

app = create_app(
    settings=setting,
    socket_client=socket_client,
    zenoh_routers=[amr_zenoh_router, move_zenoh_router],
    api_routers=[
        amr_move_router,
        amr_control_router,
        amr_localization_router,
        amr_map_router,
        amr_network_router,
        amr_setting_router,
        amr_update_router
    ],
    socket_routers=[],
    # bg_tasks=[]
    # bg_tasks=[move_mongo_service.moveDBinit],  # lifespan에서 실행됨
)



