from rb_zenoh.router import ZenohRouter

zenoh_state_router = ZenohRouter(prefix="example")


@zenoh_state_router.subscribe("state")
async def on_state(*, topic, mv, obj, attachment):
    print("👀 state")
