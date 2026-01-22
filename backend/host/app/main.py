from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.network.adapter.input.network_api import network_router
from app.features.network.adapter.input.network_zenoh import network_zenoh_router

setting = AppSettings()
app = create_app(
    settings=setting,
    socket_client=None,
    zenoh_routers=[network_zenoh_router],
    api_routers=[network_router],
    socket_routers=[],
)
