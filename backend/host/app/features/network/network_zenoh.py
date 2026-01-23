"""
[Email Zenoh 어댑터]
"""
from typing import Any
from rb_utils.parser import t_to_dict
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from app.features.network.network_service import (
    NetworkService,  # pylint: disable=import-error,no-name-in-module
)

from rb_flat_buffers.IPC.Request_Network_GetNetwork import (
    Request_Network_GetNetworkT,
)
from rb_flat_buffers.IPC.Response_Network_GetNetwork import Response_Network_GetNetworkT
from rb_flat_buffers.IPC.Request_Network_SetNetwork import Request_Network_SetNetworkT
from rb_flat_buffers.IPC.Response_Network_SetNetwork import Response_Network_SetNetworkT
from rb_flat_buffers.IPC.Request_Network_ConnectWifi import Request_Network_ConnectWifiT
from rb_flat_buffers.IPC.Response_Network_ConnectWifi import Response_Network_ConnectWifiT
from rb_flat_buffers.IPC.Request_Network_GetWifiList import Request_Network_GetWifiListT
from rb_flat_buffers.IPC.Response_Network_GetWifiList import Response_Network_GetWifiListT

network_zenoh_router = ZenohRouter()
network_service = NetworkService()

@network_zenoh_router.queryable("network/current", flatbuffer_res_buf_size=2048)
async def on_query_network() -> Response_Network_GetNetworkT:
    return await network_service.get_network()

@network_zenoh_router.queryable("network/set", flatbuffer_req_t=Request_Network_SetNetworkT, flatbuffer_res_buf_size=2048)
async def on_query_network_set(req: Request_Network_SetNetworkT) -> Response_Network_SetNetworkT:
    return await network_service.set_network(req)

@network_zenoh_router.queryable("network/wifi/connect", flatbuffer_req_t=Request_Network_ConnectWifiT, flatbuffer_res_buf_size=2048)
async def on_query_network_wifi_connect(req: Request_Network_ConnectWifiT) -> Response_Network_ConnectWifiT:
    return await network_service.connect_wifi(req)

@network_zenoh_router.queryable(
    "network/wifi/list", flatbuffer_req_t=Request_Network_GetWifiListT, flatbuffer_res_buf_size=125)
async def on_query_network_wifi_list(req: Request_Network_GetWifiListT) -> Response_Network_GetWifiListT:
    return await network_service.get_wifi_list(req)
