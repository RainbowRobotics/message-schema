from typing import Any

from rb_zenoh.client import ZenohClient

zenoh = ZenohClient()


def _resolve_target_and_route(msg: dict[str, Any]) -> tuple[str, str]:
    target = msg.get("target")
    route = msg.get("route")

    if isinstance(target, str) and isinstance(route, str):
        return target, route

    if isinstance(route, str):
        svc, sep, tail = route.partition("/")
        if sep and svc and tail:
            return svc, tail

    raise ValueError("target/route required. or route must be '<service>/<route>'")


async def forward_to_service(msg: dict[str, Any]) -> dict[str, Any]:
    target, route = _resolve_target_and_route(msg)
    payload = msg.get("payload") or {}

    if not isinstance(payload, dict):
        raise ValueError("payload must be object")

    # ✅ Zenoh query key 규칙
    # rpc/<service>/<route>
    key = f"rpc/{target}/{route}"

    # 아래는 너희 ZenohClient의 query API에 맞춰 호출만 바꾸면 됨.
    # 핵심: query 보내고 reply를 dict로 받아 반환
    res = await zenoh.query(keyexpr=key, payload=payload, timeout=3.0)
    return res
