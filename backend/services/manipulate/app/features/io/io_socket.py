from rb_socketio import (
    RbSocketIORouter,
)
from rb_utils.parser import (
    t_to_dict,
    to_json,
)

from .io_module import (
    IoService,
)

io_socket_router = RbSocketIORouter()
io_service = IoService()


@io_socket_router.on("{robot_model}/call_side_dout")
def on_call_side_dout(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = io_service.side_dout(robot_model, dict_data["port_num"], dict_data["desired_out"])
    return to_json(res)


@io_socket_router.on("{robot_model}/call_side_aout")
def on_call_side_aout(data, robot_model: str):
    dict_data = t_to_dict(data)
    res = io_service.side_aout(robot_model, dict_data["port_num"], dict_data["desired_voltage"])
    return to_json(res)
