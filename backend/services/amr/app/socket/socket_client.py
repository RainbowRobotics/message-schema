from rb_socketio import RBSocketIONsClient
socket_client = RBSocketIONsClient(
    "amr",
    reconnection=True,  # 자동 재연결 활성화
    reconnection_attempts=0,  # 0 = 무제한
    reconnection_delay=1,  # 최초 재시도 간격(초)
    reconnection_delay_max=30,  # 백오프 상한
)
