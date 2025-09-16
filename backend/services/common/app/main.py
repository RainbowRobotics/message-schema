from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rb_zenoh import zenoh_router

from app.api.demo.demo_api_route import demo_router
from app.api.state.state_api_route import state_router
from app.socket import RelayNS, app_with_sio, sio


@asynccontextmanager
async def lifespan(app: FastAPI):
    print("📡 zenoh 연결 시작", flush=True)
    await zenoh_router.startup()

    try:
        yield
    finally:
        try:
            await zenoh_router.shutdown()
            print("⛔ zenoh 연결 종료", flush=True)
        except Exception as e:
            print(f"zenoh shutdown ignore: {e}", flush=True)


app = FastAPI(root_path="/common", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


app.add_route("/socket.io/", route=app_with_sio, methods=["GET", "POST", "OPTIONS"])
app.add_websocket_route("/socket.io/", app_with_sio)


sio.register_namespace(RelayNS("/"))


app.include_router(demo_router)
app.include_router(state_router)
