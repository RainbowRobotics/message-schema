from rb_modules.log import rb_log
from rb_socketio import RBSocketIONsClient

socket_client = RBSocketIONsClient(
    "deploy",
    reconnection=True,  # 자동 재연결 활성화
    reconnection_attempts=0,  # 0 = 무제한
    reconnection_delay=1,  # 최초 재시도 간격(초)
    reconnection_delay_max=30,  # 백오프 상한
)


@socket_client.event
def connect():
    rb_log.debug("deploy server connected to socket-server")


@socket_client.event
def disconnect():
    rb_log.debug("deploy server disconnected from socket-server")


@socket_client.event
def message(data):
    rb_log.debug("deploy server received message from socket-server")
