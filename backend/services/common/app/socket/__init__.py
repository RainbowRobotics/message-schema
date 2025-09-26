import os

import socketio
from rb_socketio import RBSocketIONsClient

_REDIS_URL = os.getenv("REDIS_URL", "redis://redis:6379")
_SERVICE_SIDS: dict[str, str] = {"manipulate": None, "mobility": None, "sensor": None}

_mgr = socketio.AsyncRedisManager(_REDIS_URL)

# ALLOWED_ROOM_PREFIXES = tuple(
#     os.getenv("ALLOWED_ROOM_PREFIXES", "example-,robot:,org:,user:").split(",")
# )


# def is_room_allowed(room: str) -> bool:
#     return any(room.startswith(pfx) for pfx in ALLOWED_ROOM_PREFIXES)


def _route_service_by_event(event: str, data: dict | None) -> str | None:
    for serviceName in _SERVICE_SIDS:
        if event.startswith(f"{serviceName}.") or event.startswith(f"{serviceName}/"):
            return serviceName

    return (data or {}).get("targetService")


def _parse_auth(auth: dict | None):
    auth = auth or {}
    # if not DEV_MODE:
    #     # TODO: 나중에 실제 JWT 검증 추가 (pyjwt)
    #     # claims = jwt.decode(auth.get("token"), JWT_SECRET, algorithms=["HS256"])
    #     # return claims
    #     pass
    # DEV 전용 간이 규칙
    service = auth.get("service") or auth.get("serviceName")
    role = auth.get("role") or ("service" if service else "client")
    if service:
        return {"role": "service", "serviceName": service}
    return {"role": role}


class RelayNS(socketio.AsyncNamespace):
    async def on_connect(self, sid, environ, auth):
        try:
            claims = _parse_auth(auth)
            await self.save_session(sid, {"claims": claims})
            print(f"[ns:/] CONNECT sid={sid} claims={claims}", flush=True)

            if claims.get("role") == "service":
                svc = claims.get("serviceName")
                if not svc:
                    print("[ns:/] REJECT: serviceName required", flush=True)
                    return False

                old = _SERVICE_SIDS.get(svc)
                if old and old != sid:
                    # 중복 접속 → 정책: 새 세션 우선, 이전 세션 종료
                    print(
                        f"[ns:/] DUPLICATE for {svc}: old={old}, new={sid} → disconnect old",
                        flush=True,
                    )
                    try:
                        await self.disconnect(old)  # 이전 세션 명시 종료
                    except Exception as e:
                        print(f"[ns:/] disconnect(old) error: {e}", flush=True)

                await self.enter_room(sid, f"svc:{svc}")
                _SERVICE_SIDS[svc] = sid
                print(f"[ns:/] SET {_SERVICE_SIDS}", flush=True)

        except Exception as e:
            print(f"[ns:/] on_connect error: {type(e).__name__}: {e}", flush=True)
            return False

    async def on_disconnect(self, sid):
        sess = await self.get_session(sid)
        claims = sess.get("claims", {})
        print(f"[ns:/] on_disconnect claims={sess.get('claims')}", flush=True)
        if claims.get("role") == "service":
            svc = claims.get("serviceName")

            if _SERVICE_SIDS.get(svc) == sid:
                _SERVICE_SIDS[svc] = None

    async def on_join(self, sid, data):
        """클라: { room: 'example-state' }"""
        sess = await self.get_session(sid)
        if sess.get("claims", {}).get("role") == "service":
            return {"ok": False, "error": "service cannot join client rooms"}
        room = (data or {}).get("room") or (data or {}).get("topic")

        if not room:
            return {"ok": False, "error": "room required"}

        await self.enter_room(sid, room)
        return {"ok": True}

    async def on_leave(self, sid, data):
        """클라: { room: 'example-state' }"""
        sess = await self.get_session(sid)
        if sess.get("claims", {}).get("role") == "service":
            return {"ok": False, "error": "service cannot leave client rooms"}
        room = (data or {}).get("room") or (data or {}).get("topic")

        if not room:
            return {"ok": False, "error": "room required"}

        await self.leave_room(sid, room)
        return {"ok": True}

    async def trigger_event(self, event, sid, *args):
        if event in ("join", "leave"):
            handler = getattr(self, f"on_{event}", None)

            if handler:
                return await handler(sid, (args[0] if args else {}))

        if event == "subscribe":
            handler = getattr(self, "on_join", None)
            if handler:
                return await handler(sid, (args[0] if args else {}))

        if event == "unsubscribe":
            handler = getattr(self, "on_leave", None)
            if handler:
                return await handler(sid, (args[0] if args else {}))

        if event == "connect":
            environ = args[0] if len(args) > 0 else {}
            auth = args[1] if len(args) > 1 else None
            handler = getattr(self, "on_connect", None)
            if handler:
                return await handler(sid, environ, auth)
            return

        if event == "disconnect":
            handler = getattr(self, "on_disconnect", None)
            if handler:
                return await handler(sid)
            return

        sess = await self.get_session(sid)

        claims = sess.get("claims") or {"role": "client"}
        payload = args[0] if args else None

        service = _route_service_by_event(event, payload)
        expect_ack = bool(payload and isinstance(payload, dict) and payload.get("expectAck"))
        svc_sid = _SERVICE_SIDS.get(service)

        if claims.get("role") == "service" and not expect_ack:
            await self.emit(event, payload, room=event)
            return

        if not service:
            return {"ok": False, "error": f"no route for event '{event}'"}

        if expect_ack:
            if not svc_sid:
                return {"ok": False, "error": f"service '{service}' unavailable"}
            try:
                res = await self.server.call(svc_sid, event, payload, namespace="/", timeout=3)
                return res
            except TimeoutError:
                return {"ok": False, "error": "timeout"}
        else:
            await self.emit(event, payload, room=f"svc:{service}")
            return {"ok": True}


sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins="*",
    ping_interval=20,
    ping_timeout=45,  # 일단 30~45로 시작
    # logger=True,
    # engineio_logger=True,
)
app_with_sio = socketio.ASGIApp(sio, socketio_path="/socket.io")

socket_client = RBSocketIONsClient(
    "common",
    reconnection=True,  # 자동 재연결 활성화
    reconnection_attempts=0,  # 0 = 무제한
    reconnection_delay=1,  # 최초 재시도 간격(초)
    # logger=True,
    # engineio_logger=True,
)


@socket_client.event
def connect():
    print("common service connected to socket-server", flush=True)


@socket_client.event
def disconnect(sid):
    print("common service disconnected from socket-server", flush=True)


@socket_client.event
def message(data):
    print("common service received message from socket-server")
