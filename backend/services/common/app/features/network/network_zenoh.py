from rb_flat_buffers.IPC.Request_Network_GetNetwork import Request_Network_GetNetworkT
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions
from .network_module import NetworkService


zenoh_network_router = ZenohRouter()
network_service = NetworkService()

@zenoh_network_router.subscribe(
    "network",
    flatbuffer_obj_t=Request_Network_GetNetworkT,
    opts=SubscribeOptions(allowed_same_sender=True),
)
async def on_sub_network_response(*, topic, mv, obj, attachment):
    print("NETWORK ZENOH SUBSCRIBE")
