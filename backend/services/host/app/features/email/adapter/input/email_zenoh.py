"""
[Email Zenoh 어댑터]
"""
from rb_flat_buffers.IPC.EmailMessage import (
    EmailMessageT,
)
from rb_utils.parser import t_to_dict
from rb_zenoh.router import ZenohRouter

from app.features.email.application.email_service import (
    EmailService,
)

email_zenoh_router = ZenohRouter()
email_service = EmailService()

@email_zenoh_router.subscribe(
    "*/email",
    flatbuffer_obj_t=EmailMessageT,
)
async def on_sub_email_response(*, topic, obj):
    dict_obj = t_to_dict(obj)
    print(topic, dict_obj)
    # await email_service.send_email(dict_obj)
