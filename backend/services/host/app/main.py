from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.network.adapter.input.network_api import network_router

setting = AppSettings()
app = create_app(
    settings=setting,
    socket_client=None,
    zenoh_routers=[],
    api_routers=[network_router],
    socket_routers=[],
)
