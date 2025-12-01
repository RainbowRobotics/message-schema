from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse


class ZenohQueryException(Exception):
    pass


class ZenohReplyError(ZenohQueryException):
    def __init__(self, message: str, attachment: dict | None = None):
        super().__init__(message)
        self.attachment = attachment or {}


class ZenohNoReply(ZenohQueryException):
    def __init__(self, timeout_ms: float):
        super().__init__(f"No reply before timeout={timeout_ms}")
        self.timeout_ms = timeout_ms


class ZenohTransportError(ZenohQueryException):
    pass


def register_zenoh_exception_handlers(app: FastAPI) -> None:
    @app.exception_handler(ZenohReplyError)
    async def _reply_err(_: Request, exc: ZenohReplyError):
        app.state.rb_log.warning(f"ZenohReplyError: {exc}")
        return JSONResponse(
            status_code=502,
            content={"error": "peer error", "message": str(exc)},
        )

    @app.exception_handler(ZenohNoReply)
    async def _no_reply(_: Request, exc: ZenohNoReply):
        app.state.rb_log.warning(f"ZenohNoReply: {exc}")
        return JSONResponse(
            status_code=504,
            content={
                "error": "zenoh no reply",
                "message": f"timeout limit {exc.timeout_ms}",
            },
        )

    @app.exception_handler(ZenohTransportError)
    async def _transport(_: Request, exc: ZenohTransportError):
        app.state.rb_log.error(f"ZenohTransportError: {exc}")
        return JSONResponse(
            status_code=502,
            content={"error": "transport error", "message": str(exc)},
        )
