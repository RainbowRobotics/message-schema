from rb_zenoh import zenoh_client

# from flat_buffers.demo.example import DemoExampleHello

# from .schema import MoveCommand


def execute_move(x: float, y: float, z: float) -> dict:
    payload = {"x": x, "y": y, "z": z}
    # senderInfo = {"name": "John", "email": "john@example.com"}
    zenoh_client.publish("demo/example/hello", payload)
    return {"status": "sent", "data": payload}
