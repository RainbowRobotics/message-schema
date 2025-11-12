from rb_socketio import (
    RbSocketIORouter,
)

from .log_module import (
    LogService,
)

log_socket_router = RbSocketIORouter()
log_service = LogService()
