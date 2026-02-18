import asyncio
import socket
import sys
import unittest
from pathlib import Path
from typing import Any

PACKAGES_DIR = Path(__file__).resolve().parents[2]
SRC_PATH = PACKAGES_DIR / "rb_tcp" / "src"
src = str(SRC_PATH)
if src not in sys.path:
    sys.path.insert(0, src)

from rb_tcp.client import TcpClient, TcpClientError
from rb_tcp.gateway_server import TcpGatewayServer
from rb_tcp.registry import Registry
from rb_tcp.tcp_client_router import TcpClientRouter


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class TcpIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        print("[tcp] setup start", flush=True)
        self.registry = Registry()
        self.forward_calls: list[dict[str, Any]] = []
        self.manipulate_whoami_calls: list[dict[str, Any]] = []

        async def forwarder(msg: dict[str, Any]) -> dict[str, Any]:
            print(f"[tcp] forwarder recv={msg}", flush=True)
            self.forward_calls.append(msg)
            if msg.get("target") == "manipulate" and msg.get("route") == "whoami":
                payload = dict(msg.get("payload") or {})
                self.manipulate_whoami_calls.append(payload)
                return {
                    "service": "manipulate",
                    "route": "whoami",
                    "echo": payload,
                    "ok": True,
                }
            return {
                "echo_target": msg.get("target"),
                "echo_route": msg.get("route"),
                "echo_payload": msg.get("payload"),
            }

        self.port = _free_port()
        self.server = TcpGatewayServer(
            host="127.0.0.1",
            port=self.port,
            registry=self.registry,
            forwarder=forwarder,
            route_prefix_as_service=True,
        )
        await self.server.startup()
        print(f"[tcp] gateway up: 127.0.0.1:{self.port}", flush=True)

        self.client = TcpClient(host="127.0.0.1", port=self.port, timeout=2.0)
        await self.client.connect()
        print("[tcp] client connected", flush=True)

    async def asyncTearDown(self):
        print("[tcp] teardown start", flush=True)
        await self.client.disconnect()
        await self.server.shutdown()
        print("[tcp] teardown done", flush=True)

    async def test_ping_request_and_event_subscription(self):
        print("[tcp] test_ping_request_and_event_subscription start", flush=True)
        pong = await self.client.ping()
        self.assertTrue(pong)

        res = await self.client.request(
            target="manipulate",
            route="program/pause",
            payload={"speed": 30},
        )
        self.assertEqual(res["echo_target"], "manipulate")
        self.assertEqual(res["echo_route"], "program/pause")
        self.assertEqual(res["echo_payload"], {"speed": 30})
        self.assertEqual(len(self.forward_calls), 1)

        # target 생략 + route에 service prefix 사용
        res2 = await self.client.request(
            route="manipulate/program/resume",
            payload={"speed": 40},
        )
        self.assertEqual(res2["echo_target"], "manipulate")
        self.assertEqual(res2["echo_route"], "program/resume")
        self.assertEqual(res2["echo_payload"], {"speed": 40})
        self.assertEqual(len(self.forward_calls), 2)

        received: list[tuple[str, dict[str, Any]]] = []
        evt = asyncio.Event()

        router = TcpClientRouter()

        @router.on("robot/*/state")
        async def _on_evt(topic: str, payload: dict[str, Any]):
            received.append((topic, payload))
            evt.set()

        self.client.include_router(router)
        await self.client.subscribe("robot/r1/state")

        await self.registry.push("robot/r1/state", {"mode": "AUTO"})
        await asyncio.wait_for(evt.wait(), timeout=2.0)

        self.assertEqual(received, [("robot/r1/state", {"mode": "AUTO"})])
        print("[tcp] test_ping_request_and_event_subscription done", flush=True)

    async def test_shared_session_per_process_and_endpoint(self):
        print("[tcp] test_shared_session_per_process_and_endpoint start", flush=True)
        same = TcpClient(host="127.0.0.1", port=self.port, timeout=2.0)
        self.assertIs(self.client, same)

        await same.connect()
        self.assertTrue(self.client.connected)

        await self.client.disconnect()
        self.assertTrue(same.connected)

        await same.disconnect()
        self.assertFalse(self.client.connected)
        print("[tcp] test_shared_session_per_process_and_endpoint done", flush=True)

    async def test_service_prefix_mode_for_request_and_events(self):
        print("[tcp] test_service_prefix_mode_for_request_and_events start", flush=True)
        svc_client = TcpClient(
            host="127.0.0.1",
            port=self.port,
            timeout=2.0,
            service="manipulate",
        )
        await svc_client.connect()
        try:
            # request는 service 자동 prefix를 붙이지 않음:
            # service가 있어도 route는 명시적으로 service 포함 또는 target 지정 필요
            with self.assertRaises(TcpClientError):
                await svc_client.request(
                    route="program/pause",
                    payload={"speed": 55},
                )

            res = await svc_client.request(
                route="manipulate/program/pause",
                payload={"speed": 55},
            )
            self.assertEqual(res["echo_target"], "manipulate")
            self.assertEqual(res["echo_route"], "program/pause")
            self.assertEqual(res["echo_payload"], {"speed": 55})

            received: list[tuple[str, dict[str, Any]]] = []
            evt = asyncio.Event()

            @svc_client.on("robot/*/state")
            async def _on_prefixed(topic: str, payload: dict[str, Any]):
                received.append((topic, payload))
                evt.set()

            await svc_client.subscribe("robot/r1/state")
            await self.registry.push("manipulate/robot/r1/state", {"mode": "MANUAL"})
            await asyncio.wait_for(evt.wait(), timeout=2.0)

            self.assertEqual(
                received,
                [("manipulate/robot/r1/state", {"mode": "MANUAL"})],
            )
        finally:
            await svc_client.disconnect()
        print("[tcp] test_service_prefix_mode_for_request_and_events done", flush=True)

    async def test_external_to_common_to_manipulate_roundtrip(self):
        print("[tcp] test_external_to_common_to_manipulate_roundtrip start", flush=True)
        # 외부 클라이언트가 route="manipulate/whoami"로 요청 -> common 분기 -> manipulate 응답 가정
        res = await self.client.request(
            route="manipulate/whoami",
            payload={"from": "external", "request_id": "r1"},
        )
        self.assertEqual(res["service"], "manipulate")
        self.assertEqual(res["route"], "whoami")
        self.assertEqual(
            res["echo"],
            {"from": "external", "request_id": "r1"},
        )
        self.assertEqual(len(self.manipulate_whoami_calls), 1)
        print(
            f"[tcp] roundtrip ok response={res} calls={self.manipulate_whoami_calls}",
            flush=True,
        )
        print("[tcp] test_external_to_common_to_manipulate_roundtrip done", flush=True)


if __name__ == "__main__":
    unittest.main()
