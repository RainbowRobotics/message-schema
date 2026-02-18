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

from rb_modbus.client import ModbusClient
from rb_modbus.router import ModbusRouter
from rb_modbus.server import ModbusServer


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class ModbusIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        print("[modbus] setup start", flush=True)
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
        print(f"[modbus] server up: 127.0.0.1:{self.port}", flush=True)

        self.client = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.0)
        await self._connect_with_retry()
        print("[modbus] client connected", flush=True)

    async def asyncTearDown(self):
        print("[modbus] teardown start", flush=True)
        await self.client.disconnect()
        await self.server.shutdown()
        print("[modbus] teardown done", flush=True)

    async def test_read_and_write_holding_registers(self):
        print("[modbus] test_read_and_write_holding_registers start", flush=True)
        boot_values = await self.client.read_holding_registers(address=10, count=2, unit_id=1)
        self.assertEqual(boot_values, [1234, 5678])
        print(f"[modbus] initial values={boot_values}", flush=True)

        await self.client.write_register(address=10, value=2222, unit_id=1)
        await asyncio.wait_for(self.write_evt.wait(), timeout=2.0)

        values = await self.client.read_holding_registers(address=10, count=1, unit_id=1)
        await asyncio.wait_for(self.read_evt.wait(), timeout=2.0)

        self.assertEqual(values, [2222])
        self.assertTrue(any(e["type"] == "write" for e in self.events))
        self.assertTrue(any(e["type"] == "read" for e in self.events))
        print(f"[modbus] values={values} events={self.events}", flush=True)
        print("[modbus] test_read_and_write_holding_registers done", flush=True)

    async def test_shared_session_per_process_and_endpoint(self):
        print("[modbus] test_shared_session_per_process_and_endpoint start", flush=True)
        same = ModbusClient(host="127.0.0.1", port=self.port, timeout=2.0)
        self.assertIs(self.client, same)

        await self._connect_with_retry_for(same)
        self.assertTrue(self.client.connected)

        await self.client.disconnect()
        self.assertTrue(same.connected)

        await same.disconnect()
        self.assertFalse(self.client.connected)
        print("[modbus] test_shared_session_per_process_and_endpoint done", flush=True)

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
