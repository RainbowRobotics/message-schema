from rb_modules.rb_fastapi_app import AppSettings, create_app

from app.api.state.state_api_route import state_router
from app.api.whoami.whoami_api_route import whoami_router
from app.socket import RelayNS, app_with_sio, sio

from .zenoh_subs import zenoh_router

setting = AppSettings()

socketio_route_path = f"{setting.SOCKET_PATH}/"

app = create_app(
    settings=setting,
    zenoh_router=zenoh_router,
    api_routers=[state_router, whoami_router],
)

app.add_route(socketio_route_path, route=app_with_sio, methods=["GET", "POST", "OPTIONS"])
app.add_websocket_route(socketio_route_path, app_with_sio)


sio.register_namespace(RelayNS("/"))
