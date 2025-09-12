import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rb_zenoh.exeption import register_zenoh_exception_handlers

from app.api.state.state_api_route import state_router
from app.socket import socket_client
from app.socket.state import state_socket_router

from .zenoh_subs import zenoh_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    print("📡 subscribe 진입 전", flush=True)
    await zenoh_router.startup()
    print("📡 subscribe 진입 후", flush=True)

    app.state._sio_connect_task = asyncio.create_task(
        socket_client.connect(
            "http://localhost:11337",
            auth={"role": "service", "serviceName": "manipulate"},
            namespaces=["/"],
            socketio_path="/socket.io",
            transports=["websocket"],
            retry=True,
            wait=True,
        )
    )
    print("🔌 socket.io connect issued (non-blocking)", flush=True)

    async def _on_net_change(_cur):
        print("🔁 network change detected → restart zenoh", flush=True)
        try:
            await zenoh_router.shutdown()
        finally:
            await zenoh_router.startup()

    try:
        yield
    finally:
        try:
            await zenoh_router.shutdown()
            print("⛔ zenoh 연결 종료", flush=True)
            if getattr(socket_client, "connected", False):
                await socket_client.disconnect()
        except Exception as e:
            print(f"zenoh shutdown ignore: {e}", flush=True)


app = FastAPI(lifespan=lifespan, root_path="/manipulate")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


register_zenoh_exception_handlers(app)


app.include_router(state_router)
socket_client.socket_include_router(state_socket_router)
