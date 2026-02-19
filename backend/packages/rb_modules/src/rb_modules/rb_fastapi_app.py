import asyncio
import contextlib
import os
import sys
from collections.abc import Callable, Sequence
from contextlib import asynccontextmanager
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any

from dotenv import load_dotenv
from fastapi import APIRouter, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from pydantic import Field, model_validator
from pydantic_settings import BaseSettings
from rb_database.mongo_db import close_db, init_db
from rb_influxdb.influxdb_client import init_influxdb
from rb_modbus.client import ModbusClient
from rb_modbus.server import ModbusServer
from rb_socketio import RBSocketIONsClient, RbSocketIORouter
from rb_tcp.client import TcpClient
from rb_tcp.gateway_server import TcpGatewayServer
from rb_tcp.router import TcpRouter
from rb_tcp.rpc_zenoh import make_rpc_zenoh_router
from rb_utils.file import get_env_path
from rb_zenoh.exeption import register_zenoh_exception_handlers
from rb_zenoh.router import ZenohRouter
from starlette.exceptions import HTTPException as StarletteHTTPException

from .log import rb_log

load_dotenv(get_env_path())

# 이렇게 쓰시면 됩니당~~~
print("MONGO_USERNAME >>>", os.getenv("MONGO_USERNAME"))


class AppSettings(BaseSettings):
    IS_DEV: bool = Field(default=False)
    PORT: int | None = None
    SERVICE_NAME: str | None = None
    DATA_DIR: str | None = None

    ROOT_PATH: str | None = None

    SOCKET_PATH: str = "/socket.io"

    SOCKET_URL_DEV: str | None = "http://common:8000"
    SOCKET_URL_PROD: str | None = "http://127.0.0.1:8000"
    SOCKET_SERVER_URL: str | None = None

    MONGO_URI_DEV: str | None = "mongodb://rrs-mongo-dev:27017?replicaSet=rs0"
    MONGO_URI_PROD: str | None = "mongodb://127.0.0.1:27017?replicaSet=rs0"
    MONGO_URI: str | None = None
    MONGO_DB_NAME: str | None = "rrs"

    #influxDB
    INFLUXDB_URL_DEV: str = "http://rrs-influxdb-dev:8086"
    INFLUXDB_URL_PROD: str = "http://127.0.0.1:8086"
    INFLUXDB_URL: str | None = None
    INFLUXDB_ORG: str = "rrs"
    INFLUXDB_USERNAME: str = "rainbow"
    INFLUXDB_PASSWORD: str = "rainbow2011"
    INFLUXDB_PASSWORD_FILE: str = "/run/secrets/influxdb_password"
    INFLUXDB_TOKEN: str = "rainbowToken"
    INFLUXDB_TOKEN_FILE: str = "/run/secrets/influxdb_token"

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

        print(f">>>>>>> IS_DEV: {self.IS_DEV}", flush=True)
        print(f">>>>>>> MONGO_URI_DEV: {self.MONGO_URI_DEV}", flush=True)
        self.INFLUXDB_URL = self.INFLUXDB_URL_DEV if self.IS_DEV else self.INFLUXDB_URL_PROD
        return self


def create_app(
    *,
    settings: AppSettings,
    socket_client: RBSocketIONsClient | None = None,
    tcp_gateway: TcpGatewayServer | None = None,
    tcp_clients: Sequence[TcpClient] | None = None,
    modbus_servers: Sequence[ModbusServer] | None = None,
    modbus_clients: Sequence[ModbusClient] | None = None,
    zenoh_routers: Sequence[ZenohRouter] | None = None,
    api_routers: Sequence[APIRouter] | None = None,  # [state_router, program_router, ...]
    socket_routers: (
        Sequence[RbSocketIORouter] | None
    ) = None,  # [state_socket_router, ...] (client를 받아 등록)
    tcp_routers: Sequence[TcpRouter] | None = None,
    bg_tasks: list[Callable[[], Any]] | None = None,
) -> FastAPI:

    zenoh_router = ZenohRouter()
    tcp_router = TcpRouter()

    if bg_tasks is None:
        bg_tasks = []

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if not settings.no_init_db:
            await init_db(app, settings.MONGO_URI or "", settings.MONGO_DB_NAME or "")

        #influxdb
        try:
            if Path(settings.INFLUXDB_TOKEN_FILE).exists():
                with open(settings.INFLUXDB_TOKEN_FILE, encoding="utf-8") as f:
                    read_token = f.read().strip()
            else:
                read_token = settings.INFLUXDB_TOKEN

            await init_influxdb(app, settings.INFLUXDB_URL, read_token, settings.INFLUXDB_ORG)
        except Exception as e:
            rb_log.error(f"[InfluxDB] Initialization failed: {e}")

        await zenoh_router.startup()
        print(f"📡 zenoh subscribe 등록 완료 [PID: {os.getpid()}]", flush=True)

        if tcp_gateway is not None:
            await tcp_gateway.startup()
            app.state.tcp_gateway = tcp_gateway
            # registry 접근도 필요하면 tcp_gateway.registry 형태로 들고 있어도 됨
            print(f"🧷 tcp gateway up :{tcp_gateway.port}", flush=True)

        if tcp_clients:
            app.state.tcp_clients = list(tcp_clients)
            for tcp_client in app.state.tcp_clients:
                await tcp_client.connect()
            print(f"🧷 tcp clients up x{len(app.state.tcp_clients)}", flush=True)

        if modbus_servers:
            app.state.modbus_servers = list(modbus_servers)
            for modbus_server in app.state.modbus_servers:
                await modbus_server.startup()
            print(f"🧷 modbus servers up x{len(app.state.modbus_servers)}", flush=True)

        if modbus_clients:
            app.state.modbus_clients = list(modbus_clients)
            for modbus_client in app.state.modbus_clients:
                await modbus_client.connect()
            print(f"🧷 modbus clients up x{len(app.state.modbus_clients)}", flush=True)

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

                if getattr(app.state, "tcp_gateway", None) is not None:
                    await app.state.tcp_gateway.shutdown()
                    print("⛔ tcp gateway 종료", flush=True)

                for tcp_client in getattr(app.state, "tcp_clients", []):
                    await tcp_client.disconnect()

                for modbus_client in getattr(app.state, "modbus_clients", []):
                    await modbus_client.disconnect()

                for modbus_server in getattr(app.state, "modbus_servers", []):
                    await modbus_server.shutdown()

                if socket_client:
                    await socket_client.disconnect()

                running_bg_tasks = list(getattr(app.state, "bg_tasks", []))
                for task in running_bg_tasks:
                    task.cancel()
                for task in running_bg_tasks:
                    with contextlib.suppress(asyncio.CancelledError):
                        await task

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

    @app.middleware("http")
    async def log_unhandled_http_exception(request: Request, call_next):
        try:
            return await call_next(request)
        except Exception as exc:
            rb_log.error(
                f"[middleware] uncaught exception method={request.method} url={request.url} exc={exc}"
            )
            raise

    register_zenoh_exception_handlers(app)

    if tcp_routers:
        for tcp_r in tcp_routers:
            tcp_router.include_router(tcp_r)

        rpc_zenoh_router = make_rpc_zenoh_router(
            service=settings.SERVICE_NAME,
            tcp_router=tcp_router,
        )

        zenoh_router.include_router(rpc_zenoh_router)

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
            title=f"{(settings.SERVICE_NAME or '').capitalize()} Service",
            swagger_css_url=f"{prefix}/swagger-ui/swagger-ui.css",
            swagger_js_url=f"{prefix}/swagger-ui/swagger-ui.js",
        )

    # SDK 문서 설정
    # 실행 환경(소스/컨테이너/PyInstaller)에 따라 문서 경로를 유연하게 탐색한다.
    def _resolve_docs_html_dir() -> Path:
        candidates: list[Path] = []

        # PyInstaller(onefile): add-data "documents:documents"는 _MEIPASS 아래에 풀린다.
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(Path(meipass) / "documents" / "sdk" / "docs" / "build" / "html")

        current_file = Path(__file__).resolve()

        # 현재 파일 기준 상위 경로에서 backend/documents 탐색
        for parent in [current_file.parent, *current_file.parents]:
            candidates.append(parent / "backend" / "documents" / "sdk" / "docs" / "build" / "html")
            candidates.append(parent / "documents" / "sdk" / "docs" / "build" / "html")

        # 실행 위치 기준 fallback
        cwd = Path.cwd()
        candidates.append(cwd / "backend" / "documents" / "sdk" / "docs" / "build" / "html")
        candidates.append(cwd / "documents" / "sdk" / "docs" / "build" / "html")

        for candidate in candidates:
            if candidate.exists():
                return candidate

        # 못 찾은 경우에도 디버깅 가능한 기본 경로 반환
        return candidates[0]

    docs_html_dir = _resolve_docs_html_dir()

    @app.get("/sdk/docs", include_in_schema=False)
    def sdk_docs_root(request: Request):
        prefix = (request.scope.get("root_path") or "").rstrip("/")
        return RedirectResponse(url=f"{prefix}/sdk/docs/", status_code=307)

    @app.get("/sdk/docs/{doc_path:path}", include_in_schema=False)
    def sdk_docs_assets(doc_path: str = ""):
        if not docs_html_dir.exists():
            raise HTTPException(
                status_code=404,
                detail=f"SDK docs directory not found: {docs_html_dir}",
            )

        normalized = doc_path.strip("/")
        target_rel = "index.html" if normalized == "" else normalized
        target = (docs_html_dir / target_rel).resolve()

        try:
            target.relative_to(docs_html_dir.resolve())
        except ValueError as e:
            raise HTTPException(status_code=404, detail="Invalid docs path") from e

        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail=f"SDK docs file not found: {target_rel}")

        return FileResponse(str(target))

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

            rb_log.error(
                f"Internal Server Error body: {b.decode('utf-8', 'ignore') if isinstance(b, bytes) else b}"
            )
            return JSONResponse(
                status_code=exc.status_code if hasattr(exc, "status_code") else 500,
                content={
                    "code": exc.status_code if hasattr(exc, "status_code") else 500,
                    "error": str(exc),
                    "method": request.method,
                    "url": str(request.url),
                    "query_params": dict(request.query_params),
                    "body": body_text,
                    "message": exc.detail if hasattr(exc, "detail") else "Internal Server Error",
                },
            )
        except Exception as e:
            print("fucking exception", flush=True)
            rb_log.error(f"Internal Server Error: {e}")
            return JSONResponse(
                status_code=500,
                content={"error": "Internal Server Error", "message": str(e)},
            )

    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        return await global_exeption_handler(request, exc)

    @app.exception_handler(StarletteHTTPException)
    async def starlette_http_exception_handler(request: Request, exc: StarletteHTTPException):
        return await global_exeption_handler(request, exc)

    return app
