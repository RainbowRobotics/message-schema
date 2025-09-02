from rb_zenoh.router import ZenohRouter

zenoh_demo_router = ZenohRouter(prefix="demo")


@zenoh_demo_router.subscribe("example/hello")
async def on_demo_example_hello(*, topic, mv, obj, attachment):
    print(f"👀 demo/example/hello: {topic}, {mv}, {obj}, {attachment}")
