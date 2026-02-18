import sys
import unittest
from pathlib import Path

SERVICE_DIR = Path(__file__).resolve().parents[1]
if str(SERVICE_DIR) not in sys.path:
    sys.path.insert(0, str(SERVICE_DIR))
RB_MODBUS_SRC = SERVICE_DIR.parents[1] / "packages" / "rb_modbus" / "src"
if str(RB_MODBUS_SRC) not in sys.path:
    sys.path.insert(0, str(RB_MODBUS_SRC))

from app.tcp.tcp_forwarder import ModbusProxyConfig, build_forward_to_service


class _FakeModbusClient:
    async def read_holding_registers(self, *, address: int, count: int, unit_id: int):
        return [address, count, unit_id]

    async def write_register(self, *, address: int, value: int, unit_id: int):
        _ = (address, value, unit_id)


class TcpForwarderModbusProxyTest(unittest.IsolatedAsyncioTestCase):
    async def test_modbus_proxy_read_and_write(self):
        forward = build_forward_to_service(
            modbus_client=_FakeModbusClient(),
            modbus_proxy_config=ModbusProxyConfig(enabled=True, default_unit_id=9),
        )

        read_res = await forward(
            {
                "type": "req",
                "target": "modbus",
                "route": "read_holding_registers",
                "payload": {"address": 10, "count": 2},
                "_rb_ctx": {"session_id": "s1"},
            }
        )
        self.assertEqual(read_res, {"values": [10, 2, 9]})

        write_res = await forward(
            {
                "type": "req",
                "target": "modbus",
                "route": "write_register",
                "payload": {"address": 12, "value": 77, "unit_id": 3},
                "_rb_ctx": {"session_id": "s1"},
            }
        )
        self.assertEqual(write_res, {"ok": True})

    async def test_modbus_proxy_auth_toggle(self):
        forward = build_forward_to_service(
            modbus_client=_FakeModbusClient(),
            modbus_proxy_config=ModbusProxyConfig(enabled=True, default_unit_id=1, auth_enabled=True),
            modbus_auth_provider=lambda ctx: ctx.get("token") == "t1",
        )

        with self.assertRaises(ValueError):
            await forward(
                {
                    "type": "req",
                    "target": "modbus",
                    "route": "read_holding_registers",
                    "payload": {"address": 1, "count": 1},
                    "_rb_ctx": {"session_id": "s-auth"},
                }
            )

        ok = await forward(
            {
                "type": "req",
                "target": "modbus",
                "route": "auth",
                "payload": {"token": "t1"},
                "_rb_ctx": {"session_id": "s-auth"},
            }
        )
        self.assertEqual(ok, {"ok": True})

        read_res = await forward(
            {
                "type": "req",
                "target": "modbus",
                "route": "read_holding_registers",
                "payload": {"address": 1, "count": 1},
                "_rb_ctx": {"session_id": "s-auth"},
            }
        )
        self.assertEqual(read_res, {"values": [1, 1, 1]})


if __name__ == "__main__":
    unittest.main()
