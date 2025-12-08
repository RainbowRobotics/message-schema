import asyncio
import contextlib
import os
from collections.abc import Callable, Sequence
from contextlib import asynccontextmanager
from importlib.resources import as_file, files
from typing import Any

from dotenv import load_dotenv
from fastapi import APIRouter, FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.responses import FileResponse, JSONResponse
from pydantic import Field, model_validator
from pydantic_settings import BaseSettings
from rb_database.mongo_db import close_db, init_db
from rb_socketio import RBSocketIONsClient, RbSocketIORouter
from rb_utils.file import get_env_path
from rb_zenoh.exeption import register_zenoh_exception_handlers
from rb_zenoh.router import ZenohRouter

from .log import rb_log

load_dotenv(get_env_path())

# 이렇게 쓰시면 됩니당~~~
print("MONGO_USERNAME >>>", os.getenv("MONGO_USERNAME"))

class AppSettings(BaseSettings):
    IS_DEV: bool = Field(default=False)
    PORT: int | None = None
    SERVICE_NAME: str | None = None

    ROOT_PATH: str | None = None

    SOCKET_PATH: str = "/socket.io"

    SOCKET_URL_DEV: str | None = "http://common:8000"
    SOCKET_URL_PROD: str | None = "http://127.0.0.1:8000"
    SOCKET_SERVER_URL: str | None = None

    MONGO_URI_DEV: str | None = "mongodb://rrs-mongo-dev:27017?replicaSet=rs0"
    MONGO_URI_PROD: str | None = "mongodb://127.0.0.1:27017?replicaSet=rs0"
    MONGO_URI: str | None = None
    MONGO_DB_NAME: str | None = "rrs"

    no_init_db: bool = False
    socket_role: str = "service"

    class Config:
        env_file = "config.env"
        case_sensitive = False

    @model_validator(mode="after")
    def _compute_config(self):
        self.ROOT_PATH = f"/{self.SERVICE_NAME}"

        self.SOCKET_SERVER_URL = self.SOCKET_URL_DEV if self.IS_DEV else self.SOCKET_URL_PROD
        self.MONGO_URI = self.MONGO_URI_DEV if self.IS_DEV else self.MONGO_URI_PROD
        return self


def create_app(
    *,
    settings: AppSettings,
    socket_client: RBSocketIONsClient | None = None,
    zenoh_routers: Sequence[ZenohRouter] | None = None,
    api_routers: Sequence[APIRouter] | None = None,  # [state_router, program_router, ...]
    socket_routers: (
        Sequence[RbSocketIORouter] | None
    ) = None,  # [state_socket_router, ...] (client를 받아 등록)
    bg_tasks: list[Callable[[], Any]] | None = None,
) -> FastAPI:

    zenoh_router = ZenohRouter()

    if bg_tasks is None:
        bg_tasks = []

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if not settings.no_init_db:
            await init_db(app, settings.MONGO_URI or "", settings.MONGO_DB_NAME or "")

        await zenoh_router.startup()
        print(f"📡 zenoh subscribe 등록 완료 [PID: {os.getpid()}]", flush=True)

        if socket_client and not getattr(socket_client, "connected", False):
            app.state._sio_connect_task = asyncio.create_task(
                socket_client.connect(
                    settings.SOCKET_SERVER_URL,
                    auth=lambda: {
                        "role": settings.socket_role,
                        "serviceName": (
                            settings.SERVICE_NAME if settings.socket_role == "service" else None
                        ),
                    },
                    socketio_path=settings.SOCKET_PATH,
                    transports=["websocket", "polling"],
                    retry=True,
                    wait=True,
                )
            )

            app.state.rb_log = rb_log

            print("🔌 socket.io connect issued (non-blocking)", flush=True)

        if bg_tasks:
            for task_fn in bg_tasks:
                if asyncio.iscoroutinefunction(task_fn):
                    app_task = asyncio.create_task(task_fn())
                elif isinstance(task_fn, asyncio.Task):
                    app_task = task_fn
                else:
                    raise TypeError(f"Invalid bg_task type: {type(task_fn)}")

                app.state.bg_tasks = app.state.bg_tasks if hasattr(app.state, "bg_tasks") else []
                app.state.bg_tasks.append(app_task)

        try:
            yield
        finally:
            try:
                await zenoh_router.shutdown()
                print("⛔ zenoh 연결 종료", flush=True)

                if socket_client:
                    await socket_client.disconnect()

                for t in app.state.bg_tasks:
                    t.cancel()
                for t in bg_tasks:
                    if isinstance(t, asyncio.Task):
                        with contextlib.suppress(asyncio.CancelledError):
                            await t
                    else:
                        t()

                await close_db(app)
            except Exception as e:
                print(f"zenoh shutdown ignore: {e}", flush=True)

    app = FastAPI(
        lifespan=lifespan,
        root_path=settings.ROOT_PATH or "",
        title=f"{(settings.SERVICE_NAME or '').capitalize()} Service",
        docs_url=None,
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    register_zenoh_exception_handlers(app)

    if zenoh_routers:
        for zenoh_r in zenoh_routers:
            zenoh_router.include_router(zenoh_r)

    if api_routers:
        for api_r in api_routers:
            app.include_router(api_r)

    if socket_routers:
        for socket_r in socket_routers:
            if socket_client is not None:
                socket_client.socket_include_router(socket_r)

    @app.get("/swagger-ui/swagger-ui.css", include_in_schema=False)
    def _css():
        res = files("rb_resources").joinpath("swagger-ui", "swagger-ui.css")
        with as_file(res) as p:
            return FileResponse(str(p), media_type="text/css")

    @app.get("/swagger-ui/swagger-ui.js", include_in_schema=False)
    def _js():
        res = files("rb_resources").joinpath("swagger-ui", "swagger-ui.js")
        with as_file(res) as p:
            return FileResponse(str(p), media_type="application/javascript")

    @app.get("/docs", include_in_schema=False)
    def swagger_docs():
        prefix = (app.root_path or "").rstrip("/")

        return get_swagger_ui_html(
            openapi_url=f"{prefix}/openapi.json",
            title=f"{(settings.SERVICE_NAME or "").capitalize()} Service",
            swagger_css_url=f"{prefix}/swagger-ui/swagger-ui.css",
            swagger_js_url=f"{prefix}/swagger-ui/swagger-ui.js",
        )

    @app.exception_handler(Exception)
    async def global_exeption_handler(request: Request, exc: Exception):
        try:
            b: str | bytes = ""
            body_text: str = ""

            if request.scope.get("type") == "http":
                try:
                    b = await request.body()
                    body_text = b.decode("utf-8", "ignore") if b else ""
                    rb_log.error(f"bodytext: {body_text}")
                except RuntimeError as e:
                    rb_log.error(f"RuntimeError: {e}")
                    body_text = "<unavailable>"
                except Exception:
                    body_text = "<error reading body>"

            rb_log.error(f"Internal Server Error body: {b.decode('utf-8', 'ignore') if isinstance(b, bytes) else b}")
            return JSONResponse(
                status_code=exc.status_code if hasattr(exc, "status_code") else 500,
                content={
                    "error": exc.message if hasattr(exc, "message") else "Internal Server Error",
                    "method": request.method,
                    "url": str(request.url),
                    "query_params": dict(request.query_params),
                    "body": body_text,
                    "message": str(exc),
                },
            )
        except Exception as e:
            rb_log.error(f"Internal Server Error: {e}")
            return JSONResponse(
                status_code=500,
                content={"error": "Internal Server Error", "message": str(e)},
            )

    return app
