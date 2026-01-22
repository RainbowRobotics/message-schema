from rb_zenoh.router import ZenohRouter

from .router import TcpRouter


def make_rpc_zenoh_router(*, service: str, tcp_router: TcpRouter) -> ZenohRouter:
    """
    Zenoh query key: rpc/<service>/<route>
    예: rpc/manipulate/program/pause
    """
    r = ZenohRouter()
    prefix = f"rpc/{service}/"

    @r.queryable(f"rpc/{service}/**")
    async def _on_rpc(*, topic, payload=None, **kwargs):
        # topic이 keyexpr 문자열이라고 가정. (너희 콜백 인자명이 다르면 여기만 맞추면 됨)
        keyexpr = str(topic)
        if not keyexpr.startswith(prefix):
            return {"ok": False, "error": "bad_rpc_prefix", "keyexpr": keyexpr}

        route = keyexpr[len(prefix):]  # "program/pause"
        body = payload or {}
        if not isinstance(body, dict):
            body = {"value": body}

        return await tcp_router.dispatch(route, body)

    return r
