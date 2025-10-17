from flat_buffers.IPC.Request_SideAout_General import Request_SideAout_GeneralT
from flat_buffers.IPC.Request_SideDout_General import Request_SideDout_GeneralT
from flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh.client import ZenohClient

zenoh_client = ZenohClient()


class IoService:
    def __init__(self):
        pass

    def side_dout(self, robot_model: str, port_num: int, desired_out: int):
        req = Request_SideDout_GeneralT()
        req.port_num = port_num
        req.desired_out = desired_out

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    def side_aout(self, robot_model: str, port_num: int, desired_voltage: float):
        req = Request_SideAout_GeneralT()
        req.port_num = port_num
        req.desired_voltage = desired_voltage

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_aout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]
