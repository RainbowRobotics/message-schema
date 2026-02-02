from rb_modules.rb_fastapi_app import (
    AppSettings,
    create_app,
)

from app.features.network.network_zenoh import network_zenoh_router
from app.features.sound.sound_zenoh import sound_zenoh_router
from app.features.mdns.mdns_service import MdnsService

setting = AppSettings()
mdns_service = MdnsService()

app = create_app(
    settings=setting,
    socket_client=None,
    zenoh_routers=[network_zenoh_router, sound_zenoh_router],
    api_routers=[],
    socket_routers=[],
    bg_tasks=[mdns_service.start],
)
