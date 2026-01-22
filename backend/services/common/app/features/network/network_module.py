
from rb_modules.service import BaseService
from rb_zenoh.client import ZenohClient

from rb_flat_buffers.IPC.Request_Network_GetNetwork import (
    Request_Network_GetNetworkT,
)

from rb_flat_buffers.IPC.Response_Network_GetNetwork import (
    Response_Network_GetNetworkT,
)


zenoh_client = ZenohClient()

class NetworkService(BaseService):
    """
    Network Service
    """

    async def get_network(self):
        """
        [현재 네트워크 조회(이더넷,와이파이,블루투스)]
        """

        result = zenoh_client.query_one(
            "network",
            flatbuffer_req_obj=Request_Network_GetNetworkT(),
            flatbuffer_res_T_class=Response_Network_GetNetworkT,
            flatbuffer_buf_size=128,
        )
        return result["obj_payload"]

        # zenoh_client.publish("network", payload="HI")
        # return "HELLO"
