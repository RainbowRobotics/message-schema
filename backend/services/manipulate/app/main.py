from rb_modules.rb_fastapi_app import AppSettings, create_app
from rb_zenoh import zenoh_router

from app.api.program.program_api_route import program_router
from app.api.state.state_api_route import state_router
from app.socket import socket_client
from app.socket.program import program_socket_router
from app.socket.state import state_socket_router

app = create_app(
    settings=AppSettings(),
    socket_client=socket_client,
    zenoh_router=zenoh_router,
    api_routers=[state_router, program_router],
    socket_routers=[state_socket_router, program_socket_router],
)
