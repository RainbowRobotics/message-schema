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

network_zenoh_router = ZenohRouter()
network_service = NetworkService()


# @network_zenoh_router.subscribe(
#     "network",
#     flatbuffer_obj_t=Request_Network_GetNetworkT,
#     opts=SubscribeOptions(allowed_same_sender=True),
# )
# async def on_sub_network(*, topic, mv, obj: Request_Network_GetNetworkT, attachment):
#     print(f"HOST NETWORK ZENOH SUBSCRIBE topic={topic}")
#     return await get_network_service().get_network()

@network_zenoh_router.queryable("network")
async def on_query_network(params: dict[str, str]):
    return await network_service.get_network()
