import asyncio

from rb_modules.rb_fastapi_app import AppSettings, create_app

from app.api.info import info_router
from app.api.state import state_router
from app.api.whoami import whoami_router
from app.modules.state.state_module_service import StateService
from app.modules.whomai.whoami_module_service import WhoamiService
from app.socket import RelayNS, app_with_sio, sio, socket_client
from app.socket.whoami import whoami_socket_router
from app.zenoh_subs.state import zenoh_state_router

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
