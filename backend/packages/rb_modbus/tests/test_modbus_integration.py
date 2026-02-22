import asyncio
import socket
import sys
import unittest
from pathlib import Path

PACKAGES_DIR = Path(__file__).resolve().parents[2]
SRC_PATH = PACKAGES_DIR / "rb_modbus" / "src"
src = str(SRC_PATH)
if src not in sys.path:
    sys.path.insert(0, src)

from rb_modbus.client import ModbusClient, ModbusClientError
from rb_modbus.router import ModbusRouter
from rb_modbus.server import ModbusServer


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class ModbusIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        print("[modbus] 테스트 준비 시작", flush=True)
        self.port = _free_port()
        self.events: list[dict] = []
        self.write_evt = asyncio.Event()
        self.read_evt = asyncio.Event()

        self.router = ModbusRouter()

        @self.router.on_write("holding_registers", start=0, count=100)
        async def _on_write(payload: dict):
            self.events.append(payload)
            self.write_evt.set()

        @self.router.on_read("holding_registers", start=0, count=100)
        async def _on_read(payload: dict):
            self.events.append(payload)
            self.read_evt.set()

        self.server = ModbusServer(
            host="127.0.0.1",
            port=self.port,
            router=self.router,
            device_id=1,
            initial_holding_registers={
                10: 1234,
                11: 5678,
            },
        )
        await self.server.startup()
        print(f"[modbus] 서버 기동 완료: 127.0.0.1:{self.port}", flush=True)

        self.client = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.0)
        await self._connect_with_retry()
        print("[modbus] 클라이언트 연결 완료", flush=True)

    async def asyncTearDown(self):
        print("[modbus] 테스트 정리 시작", flush=True)
        await self.client.disconnect()
        await self.server.shutdown()
        print("[modbus] 테스트 정리 완료", flush=True)

    async def test_read_and_write_holding_registers(self):
        print("[modbus] [시나리오] Holding Register 읽기/쓰기 검증 시작", flush=True)
        boot_values = await self.client.read_holding_registers(address=10, count=2, unit_id=1)
        self.assertEqual(boot_values, [1234, 5678])
        print(f"[modbus] 초기값 확인: {boot_values}", flush=True)

        await self.client.write_register(address=10, value=2222, unit_id=1)
        await asyncio.wait_for(self.write_evt.wait(), timeout=2.0)

        values = await self.client.read_holding_registers(address=10, count=1, unit_id=1)
        await asyncio.wait_for(self.read_evt.wait(), timeout=2.0)

        self.assertEqual(values, [2222])
        self.assertTrue(any(e["type"] == "write" for e in self.events))
        self.assertTrue(any(e["type"] == "read" for e in self.events))
        print(f"[modbus] 최종값/이벤트 확인: values={values} events={self.events}", flush=True)
        print("[modbus] [시나리오] Holding Register 읽기/쓰기 검증 완료", flush=True)

    async def test_shared_session_per_process_and_endpoint(self):
        print("[modbus] [시나리오] 동일 프로세스 세션 공유 검증 시작", flush=True)
        same = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.0)
        self.assertIs(self.client, same)

        await self._connect_with_retry_for(same)
        self.assertTrue(self.client.connected)

        await self.client.disconnect()
        self.assertTrue(same.connected)

        await same.disconnect()
        self.assertFalse(self.client.connected)
        print("[modbus] [시나리오] 동일 프로세스 세션 공유 검증 완료", flush=True)

    async def test_multi_client_connections(self):
        print("[modbus] [시나리오] 다중 클라이언트 동시 접속 검증 시작", flush=True)

        client_a = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.0)
        client_b = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.5)
        self.assertIsNot(client_a, client_b)

        await self._connect_with_retry_for(client_a)
        await self._connect_with_retry_for(client_b)

        async def _read_one(c: ModbusClient):
            return await c.read_holding_registers(address=10, count=1, unit_id=1)

        res_a, res_b = await asyncio.gather(_read_one(client_a), _read_one(client_b))
        self.assertEqual(res_a, [1234])
        self.assertEqual(res_b, [1234])
        print(f"[modbus] 동시 읽기 결과: client_a={res_a}, client_b={res_b}", flush=True)

        await client_a.disconnect()
        await client_b.disconnect()
        print("[modbus] [시나리오] 다중 클라이언트 동시 접속 검증 완료", flush=True)

    async def test_illegal_address_error_code(self):
        print("[modbus] [시나리오] 접근 불가 주소 에러코드 검증 시작", flush=True)
        with self.assertRaises(ModbusClientError) as ctx:
            # 65535 범위 안이면서 서버 table_size(기본 10_000) 밖 주소를 사용해야
            # 서버에서 Illegal Address(예외코드 2) 응답을 검증할 수 있다.
            await self.client.read_holding_registers(address=12_000, count=1, unit_id=1)

        msg = str(ctx.exception)
        print(f"[modbus] 접근 불가 주소 응답 확인: {msg}", flush=True)
        self.assertIn("modbus_error:", msg)
        # pymodbus 예외코드 2: Illegal Data Address
        self.assertIn("exception_code=2", msg)
        print("[modbus] [시나리오] 접근 불가 주소 에러코드 검증 완료", flush=True)

    async def test_connection_error_for_closed_port(self):
        print("[modbus] [시나리오] 미기동 포트 연결 실패 검증 시작", flush=True)
        closed_port = _free_port()
        bad_client = ModbusClient(host="127.0.0.1", port=closed_port, timeout=0.2)
        with self.assertRaises(ModbusClientError) as ctx:
            await bad_client.connect()
        msg = str(ctx.exception)
        print(f"[modbus] 미기동 포트 에러 확인: {msg}", flush=True)
        self.assertTrue(
            ("failed_to_connect_modbus:" in msg) or ("modbus_connect_error:" in msg)
        )
        print("[modbus] [시나리오] 미기동 포트 연결 실패 검증 완료", flush=True)

    async def test_connection_error_for_invalid_host(self):
        print("[modbus] [시나리오] 잘못된 호스트 연결 실패 검증 시작", flush=True)
        bad_client = ModbusClient(host="256.256.256.256", port=502, timeout=0.2)
        with self.assertRaises(ModbusClientError) as ctx:
            await bad_client.connect()
        msg = str(ctx.exception)
        print(f"[modbus] 잘못된 호스트 에러 확인: {msg}", flush=True)
        self.assertTrue(
            ("failed_to_connect_modbus:" in msg) or ("modbus_connect_error:" in msg)
        )
        print("[modbus] [시나리오] 잘못된 호스트 연결 실패 검증 완료", flush=True)

    async def _connect_with_retry(self):
        await self._connect_with_retry_for(self.client)

    async def _connect_with_retry_for(self, client: ModbusClient):
        last_error: Exception | None = None
        for _ in range(20):
            try:
                await client.connect()
                return
            except Exception as exc:  # pragma: no cover - retry path
                last_error = exc
                await asyncio.sleep(0.05)

        raise RuntimeError(f"modbus client connection retry exhausted: {last_error}")


if __name__ == "__main__":
    unittest.main()
