from fastapi import APIRouter
from rb_zenoh import zenoh_client

demo_router = APIRouter()


@demo_router.get("/demo/hello/example")
async def demo_hello_example():
    zenoh_client.publish("demo/example/hello", {})
    return {"message": "Hello, World!"}
