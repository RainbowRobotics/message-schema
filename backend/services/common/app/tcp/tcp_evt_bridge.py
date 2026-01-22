from rb_tcp.registry import Registry
from rb_zenoh.router import ZenohRouter


def build_evt_bridge(*, registry: Registry) -> ZenohRouter:
    r = ZenohRouter()

    @r.subscribe("evt/**")  # 너희 rb_zenoh router 방식에 맞게 파라미터만 조정
    async def on_evt(*, topic, obj, **kwargs):
        # topic: "evt/robot/r1/state"
        tcp_topic = topic[4:] if isinstance(topic, str) and topic.startswith("evt/") else str(topic)
        if isinstance(obj, dict):
            await registry.push(tcp_topic, obj)
        else:
            await registry.push(tcp_topic, {"value": obj})

    return r
