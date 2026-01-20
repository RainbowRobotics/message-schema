from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

setting = AppSettings()
app = create_app(
    settings=setting,
    socket_client=None,
    zenoh_routers=[],
    api_routers=[],
    socket_routers=[],
)
