from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.log.log_api import (
    log_router,
)
from app.features.log.log_module import LogService
from app.features.log.log_socket import (
    log_socket_router,
)
from app.features.log.log_zenoh import (
    zenoh_log_router,
)
from app.socket.socket_client import (
    socket_client,
)

log_service = LogService()

app = create_app(
    settings=AppSettings(),
    socket_client=socket_client,
    zenoh_routers=[zenoh_log_router],
    api_routers=[log_router],
    socket_routers=[log_socket_router],
    bg_tasks=[log_service.start_count_updater],
)
