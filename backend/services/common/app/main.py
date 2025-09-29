import asyncio

from rb_modules.rb_fastapi_app import AppSettings, create_app

from app.features.info.info_api import info_router
from app.features.state.state_api import state_router
from app.features.state.state_module import StateService
from app.features.state.state_zenoh import zenoh_state_router
from app.features.whoami.whoami_api import whoami_router
from app.features.whoami.whoami_module import WhoamiService
from app.features.whoami.whoami_socket import whoami_socket_router
from app.socket.socket_client import socket_client
from app.socket.socket_server import RelayNS, app_with_sio, sio

setting = AppSettings()

socketio_route_path = f"{setting.SOCKET_PATH}/"

app = create_app(
    settings=setting,
    socket_client=socket_client,
    zenoh_routers=[zenoh_state_router],
    socket_routers=[whoami_socket_router],
    api_routers=[state_router, whoami_router, info_router],
)

app.add_route(socketio_route_path, route=app_with_sio, methods=["GET", "POST", "OPTIONS"])
app.add_websocket_route(socketio_route_path, app_with_sio)


sio.register_namespace(RelayNS("/"))

whoami_service = WhoamiService()
state_service = StateService()

asyncio.create_task(state_service.repeat_get_system_state())
