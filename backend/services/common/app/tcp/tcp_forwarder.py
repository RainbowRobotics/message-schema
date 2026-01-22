from typing import Any

from rb_zenoh.client import ZenohClient

zenoh = ZenohClient()

async def forward_to_service(msg: dict[str, Any]) -> dict[str, Any]:
    target = msg.get("target")
    route = msg.get("route")
    payload = msg.get("payload") or {}

    if not isinstance(target, str) or not isinstance(route, str):
        raise ValueError("target/route required")
    if not isinstance(payload, dict):
        raise ValueError("payload must be object")

    # ✅ Zenoh query key 규칙
    # rpc/<service>/<route>
    key = f"rpc/{target}/{route}"

    # 아래는 너희 ZenohClient의 query API에 맞춰 호출만 바꾸면 됨.
    # 핵심: query 보내고 reply를 dict로 받아 반환
    res = await zenoh.query(keyexpr=key, payload=payload, timeout=3.0)
    return res
