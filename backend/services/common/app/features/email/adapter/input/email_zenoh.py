"""
[Email Zenoh 어댑터]
"""
from rb_zenoh.router import (
    ZenohRouter,
)
from app.features.email.application.email_service import (
    EmailService,
)
from app.features.email.domain.email import (
    EmailModel,
)
from rb_utils.parser import (
    t_to_dict,
)

email_zenoh_router = ZenohRouter()
email_service = EmailService()

@email_zenoh_router.subscribe(
    "*/email",
    flatbuffer_obj_t=EmailModel,
)
async def on_sub_email_response(*, obj):
    dict_obj = t_to_dict(obj)
    await email_service.send_email(dict_obj)
