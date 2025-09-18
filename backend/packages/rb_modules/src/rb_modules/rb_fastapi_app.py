import asyncio
from collections.abc import Sequence
from contextlib import asynccontextmanager

from fastapi import APIRouter, FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import Field, model_validator
from pydantic_settings import BaseSettings
from rb_socketio import RBSocketIONsClient, RbSocketIORouter
from rb_zenoh import ZenohRouter
from rb_zenoh.exeption import register_zenoh_exception_handlers


class AppSettings(BaseSettings):
    IS_DEV: bool = Field(default=False)
    PORT: int
    SERVICE_NAME: str

    ROOT_PATH: str | None = None

    SOCKET_PATH: str = "/socket.io"

    SOCKET_URL_DEV: str | None = "http://common:8000"
    SOCKET_URL_PROD: str | None = "http://127.0.0.1:8000"
    SOCKET_SERVER_URL: str | None = None

    class Config:
        env_file = "conf.env"
        case_sensitive = False

    @model_validator(mode="after")
    def _compute_config(self):
        self.ROOT_PATH = f"/{self.SERVICE_NAME}"

        self.SOCKET_SERVER_URL = self.SOCKET_URL_DEV if self.IS_DEV else self.SOCKET_URL_PROD
        return self


def create_app(
    *,
    settings: AppSettings,
    socket_client: RBSocketIONsClient | None = None,
    zenoh_router: ZenohRouter,
    api_routers: Sequence[APIRouter] | None = None,  # [state_router, program_router, ...]
    socket_routers: (
        Sequence[RbSocketIORouter] | None
    ) = None,  # [state_socket_router, ...] (client를 받아 등록)
) -> FastAPI:

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await zenoh_router.startup()
        print("📡 zenoh subscribe 등록 완료", flush=True)

        if socket_client:
            app.state._sio_connect_task = asyncio.create_task(
                socket_client.connect(
                    settings.SOCKET_SERVER_URL,
                    auth={"role": "service", "serviceName": settings.SERVICE_NAME},
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
                if socket_client and getattr(socket_client, "connected", False):
                    await socket_client.disconnect()
            except Exception as e:
                print(f"zenoh shutdown ignore: {e}", flush=True)

    print(">>settings.ROOT_PATH", settings.ROOT_PATH)
    app = FastAPI(
        lifespan=lifespan,
        root_path=settings.ROOT_PATH,
        title=f"{settings.SERVICE_NAME.capitalize()} Service",
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    register_zenoh_exception_handlers(app)

    if api_routers:
        for r in api_routers:
            app.include_router(r)

    if socket_routers:
        for r in socket_routers:
            socket_client.socket_include_router(r)

    @app.exception_handler(Exception)
    async def global_exeption_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=500,
            content={
                "error": "Internal Server Error",
                "body": request.body,
                "query_params": request.query_params,
                "message": str(exc),
            },
        )

    return app
