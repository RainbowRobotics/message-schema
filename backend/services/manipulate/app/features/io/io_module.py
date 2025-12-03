
from rb_flat_buffers.IPC.Request_Flange_Digital_Out import Request_Flange_Digital_OutT
from rb_flat_buffers.IPC.Request_Flange_Power import Request_Flange_PowerT
from rb_flat_buffers.IPC.Request_SideAout_General import Request_SideAout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Bitcombination import Request_SideDout_BitcombinationT
from rb_flat_buffers.IPC.Request_SideDout_Pulse import Request_SideDout_PulseT
from rb_flat_buffers.IPC.Request_SideDout_Toggle import Request_SideDout_ToggleT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_zenoh.client import ZenohClient

zenoh_client = ZenohClient()



class IoService(BaseService):
    def __init__(self):
        pass

    async def call_side_dout(self, robot_model: str, port_num: int, desired_out: int):
        res = self.manipulate_sdk.call_side_dout(
            robot_model=robot_model,
            port_num=port_num,
            desired_out=desired_out,
        )
        return res

    async def call_side_aout(self, robot_model: str, port_num: int, desired_voltage: float):
        req = Request_SideAout_GeneralT()
        req.portNum = request.port_num
        req.desiredVoltage = request.desired_voltage

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_side_aout", req, Response_FunctionsT, 8
        )

        return res["dict_payload"]

    async def call_flange_power(self, robot_model: str, desired_voltage: int):
        req = Request_Flange_PowerT()
        req.desiredVoltage = request.desired_voltage

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_flange_power", req, Response_FunctionsT, 8
        )
        return res["dict_payload"]

    async def call_flange_dout(self, robot_model: str, port_num: int, desired_out: int):
        req = Request_Flange_Digital_OutT()
        req.portNum = request.port_num
        req.desiredOut = request.desired_out

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_flange_dout", req, Response_FunctionsT, 8
        )
        return res["dict_payload"]

    async def call_side_dout_toggle(self, robot_model: str, port_num: int):
        req = Request_SideDout_ToggleT()
        req.portNum = request.port_num

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_side_dout_toggle", req, Response_FunctionsT, 8
        )
        return res["dict_payload"]

    async def call_side_dout_bitcombination(
        self,
        robot_model: str,
        port_start: int,
        port_end: int,
        desired_value: int,
        direction_option: int,
    ):
        req = Request_SideDout_BitcombinationT()
        req.portStart = request.port_start
        req.portEnd = request.port_end
        req.desiredValue = request.desired_value
        req.directionOption = request.direction_option

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_side_dout_bitcombination", req, Response_FunctionsT, 16
        )
        return res["dict_payload"]

    async def call_side_dout_pulse(
        self,
        robot_model: str,
        port_num: int,
        block_mode: int,
        direction: int,
        time_1: float,
        time_2: float,
        time_3: float,
    ):
        req = Request_SideDout_PulseT()
        req.portNum = request.port_num
        req.blockMode = request.block_mode
        req.direction = request.direction
        req.time1 = request.time_1
        req.time2 = request.time_2
        req.time3 = request.time_3

        res = await self._execute_zenoh_query(
            f"{robot_model}/call_side_dout_pulse", req, Response_FunctionsT, 24
        )

        return res["dict_payload"]
