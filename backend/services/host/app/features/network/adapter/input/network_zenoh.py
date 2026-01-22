"""
[Email Zenoh 어댑터]
"""
from rb_zenoh.router import ZenohRouter

from app.features.network.application.network_service import (
    NetworkService,  # pylint: disable=import-error,no-name-in-module
)

email_zenoh_router = ZenohRouter()
network_service = NetworkService()

# @email_zenoh_router.subscribe(
#     "*/network",
#     flatbuffer_obj_t=NetworkMessageT,
# )
# async def on_sub_network_response(*, topic, obj):
#     dict_obj = t_to_dict(obj)
#     print(topic, dict_obj)
#     # await network_service.get_network(dict_obj)
