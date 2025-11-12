from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.config.config_api import (
    config_router,
)
from app.features.io.io_api import (
    io_router,
)
from app.features.io.io_socket import (
    io_socket_router,
)
from app.features.program.program_api import (
    program_router,
)
from app.features.program.program_socket import (
    program_socket_router,
)
from app.features.state.state_api import (
    state_router,
)
from app.features.state.state_socket import (
    state_socket_router,
)
from app.features.state.state_zenoh import (
    zenoh_state_router,
)
from app.socket.socket_client import (
    socket_client,
)

app = create_app(
    settings=AppSettings(),
    socket_client=socket_client,
    zenoh_routers=[zenoh_state_router],
    api_routers=[state_router, program_router, config_router, io_router],
    socket_routers=[state_socket_router, program_socket_router, io_socket_router],
)
