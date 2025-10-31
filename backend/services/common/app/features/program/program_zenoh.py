from rb_database import get_db
from rb_zenoh.router import ZenohRouter
from rb_zenoh.schema import SubscribeOptions

from .program_module import ProgramService

zenoh_program_router = ZenohRouter()

program_service = ProgramService()


@zenoh_program_router.subscribe("rrs/pause", opts=SubscribeOptions(allowed_same_sender=True))
async def on_pause(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.call_resume_or_pause(db=db, is_pause=True)


@zenoh_program_router.subscribe("rrs/resume", opts=SubscribeOptions(allowed_same_sender=True))
async def on_resume(*, topic, mv, obj, attachment):
    db = await get_db()
    return await program_service.call_resume_or_pause(db=db, is_pause=False)
