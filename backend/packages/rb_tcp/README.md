# rb_tcp 사용 설명서

`rb_tcp`는 다음 기능을 제공합니다.

- TCP 게이트웨이 서버 (`TcpGatewayServer`)
- TCP 클라이언트 (`TcpClient`)
- 서버 라우터 데코레이터 (`TcpRouter`)
- 클라이언트 이벤트 라우터 데코레이터 (`TcpClientRouter`)

## 1. 설치/의존성

워크스페이스에서는 `rb_tcp`가 이미 패키지로 연결되어 있습니다.

## 2. 기본 메시지 타입

게이트웨이 프로토콜의 주요 타입:

- `ping` -> `pong`
- `req` -> `res` (RPC 요청/응답)
- `sub` / `unsub` (토픽 구독/해제)
- `evt` (서버가 구독자에게 push)

## 2-1. 라우팅 규칙 (`target` 생략 지원)

`req` 요청 시 두 방식 모두 지원합니다.

1. 분리형
- `target="manipulate"`
- `route="program/pause"`

2. 결합형
- `target` 생략
- `route="manipulate/program/pause"`

결합형은 service prefix(`manipulate`)를 자동 분리해 `target`으로 처리합니다.

## 2-2. 연결 공유 정책

`TcpClient`는 같은 프로세스에서 `(host, port, timeout)`이 같으면 동일 인스턴스/연결을 공유합니다.

- 같은 키로 `TcpClient(...)`를 여러 번 생성해도 내부 연결은 1개만 사용
- `connect()`/`disconnect()`는 참조 카운트 방식으로 동작
- 마지막 `disconnect()`가 호출될 때 실제 소켓이 종료

## 3. 서버 만들기

```python
from rb_tcp import Registry, TcpGatewayServer

registry = Registry()

async def forwarder(msg: dict) -> dict:
    # msg: {"type":"req", "target":"service", "route":"path", "payload":{...}}
    return {"ok": True, "echo": msg.get("payload")}

gateway = TcpGatewayServer(
    host="0.0.0.0",
    port=9100,
    registry=registry,
    forwarder=forwarder,
    route_prefix_as_service=True,  # "manipulate/program/pause" -> target=manipulate, route=program/pause
)

await gateway.startup()
# ...
await gateway.shutdown()
```

`route_prefix_as_service` 옵션:
- `False` (기본): 순수 TCP 서버처럼 메시지를 그대로 `forwarder`에 전달
- `True`: `req.route`의 첫 segment를 service로 분리해서 `target`/`route`로 정규화

## 3-1. 권장 아키텍처 (common 대장 서버)

- `common` 서비스: `TcpGatewayServer` 단일 운영
- 각 서비스: `TcpClient`로 `common`에 연결
- RPC 규칙:
  - 분리형: `target=<service_name>`, `route=<service_route>`
  - 결합형: `route=<service_name>/<service_route>`
- 이벤트 fan-out: `common`이 `Registry.push()`로 전달

즉, 각 서비스가 TCP 서버를 따로 띄우는 방식보다 `common` 집중형이 권장됩니다.

## 3-2. 서비스 경계 정책 (manipulate / amr)

서비스 분리는 **route prefix**로 고정합니다.

- manipulate: `manipulate/...`
- amr: `amr/...`

권장 예시:
- manipulate 요청: `route="manipulate/whoami"`
- amr 요청: `route="amr/status/get"`

클라이언트에서 강제하려면:

```python
# manipulate 전용
manipulate_client = TcpClient(host="127.0.0.1", port=9100, service="manipulate")

# amr 전용
amr_client = TcpClient(host="127.0.0.1", port=9100, service="amr")
```

이렇게 하면 `subscribe/on`에서 각 서비스 prefix가 자동 적용됩니다.
`request`는 자동 prefix를 붙이지 않으므로 `target`을 명시하거나 `route="service/..."` 형태로 보내야 합니다.

## 4. 클라이언트 만들기

```python
from rb_tcp import TcpClient

client = TcpClient(host="127.0.0.1", port=9100)
await client.connect()

await client.ping()

res = await client.request(
    target="manipulate",
    route="program/pause",
    payload={"speed": 30},
)

# target 생략 가능 (route에 service prefix 포함)
res2 = await client.request(
    route="manipulate/program/resume",
    payload={"speed": 40},
)

await client.disconnect()
```

클라이언트 `service` 옵션:

```python
client = TcpClient(host="127.0.0.1", port=9100, service="manipulate")
```

- `subscribe("robot/r1/state")` -> `manipulate/robot/r1/state`로 구독
- `@client.on("robot/*/state")` -> `manipulate/robot/*/state` 패턴으로 수신
- `request`는 자동 prefix 미적용:
  - `request(target="manipulate", route="program/pause")`
  - 또는 `request(route="manipulate/program/pause")`

## 5. 클라이언트 이벤트 데코레이터

```python
from rb_tcp import TcpClientRouter

router = TcpClientRouter()

@router.on("robot/*/state")
async def on_state(topic: str, payload: dict):
    print(topic, payload)

client.include_router(router)
await client.subscribe("robot/r1/state")
```

## 6. `create_app`에 등록하기

`rb_modules.rb_fastapi_app.create_app`에서 바로 등록 가능합니다.

```python
app = create_app(
    settings=setting,
    tcp_gateway=tcp_gateway,      # 서버 1개
    tcp_clients=[tcp_client],     # 클라이언트 여러 개 가능
    tcp_routers=[tcp_router],     # 기존 tcp rpc router
    zenoh_routers=[...],
    api_routers=[...],
)
```

`create_app` lifespan에서 startup/shutdown을 자동 처리합니다.

각 서비스에서 common 게이트웨이 TCP 클라이언트를 붙일 때:

```python
gateway_client = TcpClient(host="127.0.0.1", port=9100, service="manipulate")

app = create_app(
    settings=AppSettings(),
    tcp_clients=[gateway_client],  # create_app이 connect/disconnect 관리
    zenoh_routers=[...],
    api_routers=[...],
)
```

## 6-1. `rb_flow_manager` Step 예제

아래 예제는 요청하신 형태처럼 **한 Step에서 연결/처리/종료까지** 끝냅니다.

### A. 서버 원샷 Step (열고, 요청 1건 받고 응답 후 닫기)


```python
import asyncio
from rb_flow_manager.step import Step
from rb_tcp import Registry, TcpGatewayServer

async def step_tcp_server_once(*, flow_manager_args):
    registry = Registry()
    got_req = asyncio.Event()
    last_req: dict | None = None

    async def forwarder(msg: dict) -> dict:
        nonlocal last_req
        last_req = msg
        got_req.set()
        return {
            "ok": True,
            "service": msg.get("target"),
            "route": msg.get("route"),
            "payload": msg.get("payload"),
        }

    server = TcpGatewayServer(
        host="0.0.0.0",
        port=9200,
        registry=registry,
        forwarder=forwarder,
        route_prefix_as_service=True,
    )

    await server.startup()
    try:
        # 외부 클라이언트 요청이 들어올 때까지 최대 10초 대기
        await asyncio.wait_for(got_req.wait(), timeout=10.0)
        print("server got request:", last_req, flush=True)
    finally:
        await server.shutdown()

    flow_manager_args.done()

tree = Step(
    step_id="tcp_server_once",
    name="tcp_server_once",
    func=step_tcp_server_once,
)
```

### B. 클라이언트 원샷 Step (연결, 요청/응답, 종료)

```python
from rb_flow_manager.step import Step
from rb_tcp import TcpClient

async def step_tcp_client_once(*, flow_manager_args):
    # service 옵션은 subscribe/on에만 자동 적용, request는 명시적으로 service를 포함해야 함
    client = TcpClient(host="127.0.0.1", port=9100, service="manipulate")
    await client.connect()
    try:
        res = await client.request(
            route="manipulate/whoami",
            payload={"from": "flow-step"},
        )
        print("client response:", res, flush=True)
    finally:
        await client.disconnect()

    flow_manager_args.done()

tree = Step(
    step_id="123...4594598",
    name="tcp_client_once",
    func=step_tcp_client_once,
)
```

## 7. 테스트 실행

```bash
make backend.test-tcp
```
