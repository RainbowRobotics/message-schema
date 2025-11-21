from rb_flat_buffers.IPC.Request_Flange_Digital_Out import Request_Flange_Digital_OutT
from rb_flat_buffers.IPC.Request_Flange_Power import Request_Flange_PowerT
from rb_flat_buffers.IPC.Request_SideAout_General import Request_SideAout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_General import Request_SideDout_GeneralT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_zenoh.client import ZenohClient

zenoh_client = ZenohClient()


class IoService:
    def __init__(self):
        pass

    async def side_dout(self, robot_model: str, port_num: int, desired_out: int):
        req = Request_SideDout_GeneralT()
        req.portNum = port_num
        req.desiredOut = desired_out

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]

    async def side_aout(self, robot_model: str, port_num: int, desired_voltage: float):
        req = Request_SideAout_GeneralT()
        req.portNum = port_num
        req.desiredVoltage = desired_voltage

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_aout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    async def flange_power(self, robot_model: str, desired_voltage: int):
        req = Request_Flange_PowerT()
        req.desiredVoltage = desired_voltage

        res = zenoh_client.query_one(
            f"{robot_model}/call_flange_power",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    async def flange_dout(self, robot_model: str, port_num: int, desired_out: int):
        req = Request_Flange_Digital_OutT()
        req.portNum = port_num
        req.desiredOut = desired_out

        res = zenoh_client.query_one(
            f"{robot_model}/call_flange_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]