from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.network.network_zenoh import network_zenoh_router

setting = AppSettings()
app = create_app(
    settings=setting,
    socket_client=None,
    zenoh_routers=[network_zenoh_router],
    api_routers=[],
    socket_routers=[],
)
