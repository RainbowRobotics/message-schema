from rb_modules.rb_fastapi_app import AppSettings, create_app

from app.features.deploy.deploy_api import deploy_router
from app.features.deploy.deploy_socket import deploy_socket_router
from app.features.file.file_api import file_router
from app.features.file.file_socket import file_socket_router
from app.socket.socket_client import socket_client

setting = AppSettings()
setting.no_init_db = True

app = create_app(
    settings=setting,
    socket_client=socket_client,
    zenoh_routers=[],
    api_routers=[deploy_router, file_router],
    socket_routers=[deploy_socket_router, file_socket_router],
)
