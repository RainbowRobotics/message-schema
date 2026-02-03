from rb_socketio import (
    RBSocketIONsClient,
)



@socket_client.event
def connect():
    print("amr service connected to socket-server", flush=True)


@socket_client.event
def disconnect(sid):
    print("amr service disconnected from socket-server", flush=True)


@socket_client.event
def message(data):
    print("amr service received message from socket-server")
