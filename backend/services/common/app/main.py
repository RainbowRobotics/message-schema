from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)
from rb_tcp.gateway_server import TcpGatewayServer
from rb_tcp.registry import Registry

from app.features.config.config_api import config_router
from app.features.config.config_module import ConfigService
from app.features.config.config_socket import config_socket_router
from app.features.email.adapter.input.email_api import (
    email_router,
)
from app.features.info.info_api import info_router
from app.features.info.info_socket import info_socket_router
from app.features.network.network_api import network_router
from app.features.program.program_api import program_router
from app.features.program.program_zenoh import zenoh_program_router
from app.features.sound.sound_api import sound_router
from app.features.state.state_api import state_router
from app.features.state.state_module import StateService
from app.features.state.state_socket import state_socket_router
from app.features.state.state_zenoh import zenoh_state_router
from app.features.whoami.whoami_api import whoami_router
from app.features.whoami.whoami_socket import whoami_socket_router
from app.socket.socket_client import socket_client
from app.socket.socket_server import RelayNS, app_with_sio, sio
from app.tcp.tcp_evt_bridge import build_evt_bridge
from app.tcp.tcp_forwarder import forward_to_service

setting = AppSettings()

registry = Registry()
tcp_gateway = TcpGatewayServer(
    host="0.0.0.0",
    port=9100,
    registry=registry,
    forwarder=forward_to_service,
)
evt_router = build_evt_bridge(registry=registry)

SOCKET_IO_ROUTE_PATH = f"{setting.SOCKET_PATH}/"


state_service = StateService()
config_service = ConfigService()
# program_service = ProgramService()


app = create_app(
    settings=setting,
    socket_client=socket_client,
    tcp_gateway=tcp_gateway,
    zenoh_routers=[zenoh_state_router, zenoh_program_router, evt_router],
    socket_routers=[
        whoami_socket_router,
        config_socket_router,
        state_socket_router,
        info_socket_router,
    ],
    api_routers=[
        state_router,
        config_router,
        whoami_router,
        info_router,
        program_router,
        email_router,
        network_router,
        sound_router,
    ],
    bg_tasks=[
        state_service.repeat_get_system_state,
        config_service.repeat_get_all_speedbar,
        # program_service.steps_watch_worker,
        # program_service.send_executor_state,
    ],
)

app.add_route(SOCKET_IO_ROUTE_PATH, route=app_with_sio, methods=["GET", "POST", "OPTIONS"])
app.add_websocket_route(SOCKET_IO_ROUTE_PATH, app_with_sio)


sio.register_namespace(RelayNS("/"))



# while True:
#     time.sleep(1)
#     zenoh_client.publish("*/pause", payload=b"1")
