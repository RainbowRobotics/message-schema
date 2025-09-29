from rb_socketio import RBSocketIONsClient

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
