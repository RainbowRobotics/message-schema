# rb_modbus 사용 설명서

`rb_modbus`는 Modbus TCP 서버/클라이언트와 데코레이터 기반 훅 라우터를 제공합니다.

## 1. 주요 기능

- Modbus TCP 서버 (`ModbusServer`)
- Modbus TCP 클라이언트 (`ModbusClient`)
- 읽기/쓰기 훅 라우터 (`ModbusRouter`)

## 1-1. 연결 공유 정책

`ModbusClient`는 같은 프로세스에서 `(host, port, timeout)`이 같으면 동일 인스턴스/연결을 공유합니다.

- 같은 키로 `ModbusClient(...)`를 여러 번 생성해도 내부 연결은 1개만 사용
- `connect()`/`disconnect()`는 참조 카운트 방식으로 동작
- 마지막 `disconnect()`가 호출될 때 실제 연결이 종료

## 2. 서버 만들기

```python
from rb_modbus import ModbusRouter, ModbusServer

router = ModbusRouter()

@router.on_write("holding_registers", start=100, count=10)
async def on_hr_write(evt: dict):
    print("write evt:", evt)

@router.on_read("holding_registers", start=100, count=10)
async def on_hr_read(evt: dict):
    print("read evt:", evt)

server = ModbusServer(
    host="0.0.0.0",
    port=1502,
    router=router,
    device_id=1,
    initial_holding_registers={100: 10, 101: 20},  # 시작 시 초기값 preload
    initial_coils={0: True, 1: False},
)

await server.startup()
# ...
await server.shutdown()
```

초기값 옵션:

- `initial_holding_registers: dict[int, int]`
- `initial_input_registers: dict[int, int]`
- `initial_coils: dict[int, bool]`
- `initial_discrete_inputs: dict[int, bool]`

모든 테이블 기본값은 0이며, 위 옵션으로 필요한 주소만 preload할 수 있습니다.

## 2-1. 서비스 경계 정책 (manipulate / amr)

Modbus는 서비스별로 **주소 범위**를 분리해서 사용하세요. (겹치지 않게 고정)

권장 기본안 (holding registers 기준):

- manipulate: `228 ~ 428`
- amr: `528 ~ 728`

예시:

```python
# manipulate 서버 예시
modbus_server_manipulate = ModbusServer(
    host="0.0.0.0",
    port=1502,
    device_id=1,
    initial_holding_registers={
        228: 1,
        238: 1234,
    },
)

# amr 서버 예시
modbus_server_amr = ModbusServer(
    host="0.0.0.0",
    port=1502,
    device_id=1,
    initial_holding_registers={
        528: 1,
        538: 5678,
    },
)
```

운영 시에는 문서/코드에서 범위를 상수로 고정해두는 것을 권장합니다.

## 3. 클라이언트 만들기

```python
from rb_modbus import ModbusClient

client = ModbusClient(host="127.0.0.1", port=1502, timeout=2.0)
await client.connect()

await client.write_register(address=10, value=1234, unit_id=1)
values = await client.read_holding_registers(address=10, count=1, unit_id=1)
print(values)  # [1234]

await client.disconnect()
```

## 4. 지원 메서드

`ModbusClient`에서 제공:

- `read_holding_registers`
- `read_input_registers`
- `read_coils`
- `write_register`
- `write_registers`
- `write_coil`
- `write_coils`

## 5. `create_app`에 등록하기

`rb_modules.rb_fastapi_app.create_app`에 바로 넘길 수 있습니다.

```python
app = create_app(
    settings=setting,
    modbus_servers=[modbus_server],
    modbus_clients=[modbus_client],
    zenoh_routers=[...],
    api_routers=[...],
)
```

`create_app` lifespan에서 startup/shutdown을 자동 처리합니다.

## 5-1. `rb_flow_manager` Step 예제

요청하신 형태처럼 **한 Step에서 연결/처리/종료까지** 끝내는 예제입니다.

### A. Modbus 서버 원샷 Step (열고, 외부 요청 대기 후 닫기)

```python
import asyncio
from rb_flow_manager.step import Step
from rb_modbus import ModbusServer

async def step_modbus_server_once(*, flow_manager_args):
    server = ModbusServer(
        host="0.0.0.0",
        port=1502,
        device_id=1,
        initial_holding_registers={10: 1234},
    )
    await server.startup()
    try:
        # 외부 클라이언트 read/write를 받을 시간 (예시 10초)
        await asyncio.sleep(10.0)
    finally:
        await server.shutdown()

    flow_manager_args.done()

tree = Step(
    step_id="modbus_server_once",
    name="modbus_server_once",
    func=step_modbus_server_once,
)
```

### B. Modbus 클라이언트 원샷 Step (연결, 요청/응답, 종료)

```python
from rb_flow_manager.step import Step
from rb_modbus import ModbusClient

async def step_modbus_client_once(*, flow_manager_args):
    client = ModbusClient(host="127.0.0.1", port=1502, timeout=2.0)
    await client.connect()
    try:
        await client.write_register(address=10, value=2222, unit_id=1)
        values = await client.read_holding_registers(address=10, count=1, unit_id=1)
        print("modbus values:", values, flush=True)
    finally:
        await client.disconnect()

    flow_manager_args.done()

tree = Step(
    step_id="modbus_client_once",
    name="modbus_client_once",
    func=step_modbus_client_once,
)
```

## 6. 테스트 실행

```bash
make backend.test-modbus
```

## 7. 인증 구성 권장

Modbus TCP 표준에는 JWT 핸드셰이크가 없습니다.  
그래서 인증은 `rb_modbus` 내부가 아니라 **앞단 TCP 게이트웨이 프록시(`rb_tcp`)**에서 세션 단위로 처리하는 구성을 권장합니다.
