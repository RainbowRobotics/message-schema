"""
[Email Zenoh 어댑터]
"""
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from app.features.network.application.network_service import (
    NetworkService,  # pylint: disable=import-error,no-name-in-module
)

from rb_flat_buffers.IPC.Request_Network_GetNetwork import (
    Request_Network_GetNetworkT,
)
from rb_flat_buffers.IPC.Response_Network_GetNetwork import Response_Network_GetNetworkT

network_zenoh_router = ZenohRouter()
network_service = NetworkService()

@network_zenoh_router.queryable("network", flatbuffer_res_buf_size=2048)
async def on_query_network() -> Response_Network_GetNetworkT:
    return await network_service.get_network()
