from rb_zenoh import ZenohRouter

from .demo import zenoh_demo_router

zenoh_router = ZenohRouter()

zenoh_router.include_router(zenoh_demo_router)