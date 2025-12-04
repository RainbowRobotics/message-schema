
from rb_flat_buffers.IPC.Request_Flange_Digital_Out import Request_Flange_Digital_OutT
from rb_flat_buffers.IPC.Request_Flange_Power import Request_Flange_PowerT
from rb_flat_buffers.IPC.Request_Save_SideDin_FilterCount import Request_Save_SideDin_FilterCountT
from rb_flat_buffers.IPC.Request_Save_SideDin_SpecialFunc import Request_Save_SideDin_SpecialFuncT
from rb_flat_buffers.IPC.Request_Save_SideDout_SpecialFunc import Request_Save_SideDout_SpecialFuncT
from rb_flat_buffers.IPC.Request_SideAout_General import Request_SideAout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Bitcombination import Request_SideDout_BitcombinationT
from rb_flat_buffers.IPC.Request_SideDout_General import Request_SideDout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Pulse import Request_SideDout_PulseT
from rb_flat_buffers.IPC.Request_SideDout_Toggle import Request_SideDout_ToggleT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_zenoh.client import ZenohClient

from .io_schema import (
    Request_Flange_Digital_OutPD,
    Request_Flange_PowerPD,
    Request_Save_SideDin_FilterCountPD,
    Request_Save_SideDin_SpecialFuncPD,
    Request_Save_SideDout_SpecialFuncPD,
    Request_SideAout_GeneralPD,
    Request_SideDout_BitcombinationPD,
    Request_SideDout_GeneralPD,
    Request_SideDout_PulsePD,
    Request_SideDout_TogglePD,
)

zenoh_client = ZenohClient()


class IoService(BaseService):
    def __init__(self):
        pass

    # ==========================================================================
    # 1. Side IO Control
    # ==========================================================================    

    async def call_side_dout(self, robot_model: str, request: Request_SideDout_GeneralPD):
        req = Request_SideDout_GeneralT()
        req.portNum = request.port_num
        req.desiredOut = request.desired_out

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout_toggle",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]


    async def call_side_aout(self, robot_model: str, request: Request_SideAout_GeneralPD):
        req = Request_SideAout_GeneralT()
        req.portNum = request.port_num
        req.desiredVoltage = request.desired_voltage

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_aout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        return res["dict_payload"]


    async def call_side_dout_toggle(self, robot_model: str, request: Request_SideDout_TogglePD):
        req = Request_SideDout_ToggleT()
        req.portNum = request.port_num

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout_toggle",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]


    async def call_side_dout_bitcombination(self, robot_model: str, request: Request_SideDout_BitcombinationPD):
        req = Request_SideDout_BitcombinationT()
        req.portStart = request.port_start
        req.portEnd = request.port_end
        req.desiredValue = request.desired_value
        req.directionOption = request.direction_option

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout_bitcombination",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=16,
        )
        return res["dict_payload"]


    async def call_side_dout_pulse(self, robot_model: str, request: Request_SideDout_PulsePD):
        req = Request_SideDout_PulseT()
        req.portNum = request.port_num
        req.blockMode = request.block_mode
        req.direction = request.direction
        req.time1 = request.time_1
        req.time2 = request.time_2
        req.time3 = request.time_3

        res = zenoh_client.query_one(
            f"{robot_model}/call_side_dout_pulse",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=24,
        )

        return res["dict_payload"]

    # ==========================================================================
    # 2. IO Configuration (DIN/DOUT)
    # ==========================================================================

    async def save_side_dout_function(
        self, robot_model: str, *, request: Request_Save_SideDout_SpecialFuncPD):
        req = Request_Save_SideDout_SpecialFuncT()
        req.portNum = request.port_num
        req.desiredFunction = request.desired_function

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_dout_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]


    async def save_side_din_function(self, robot_model: str, *, request: Request_Save_SideDin_SpecialFuncPD):
        req = Request_Save_SideDin_SpecialFuncT()
        req.portNum = request.port_num
        req.desiredFunction = request.desired_function

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_function",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]


    async def save_side_din_filter(self, robot_model: str, *, request: Request_Save_SideDin_FilterCountPD):
        req = Request_Save_SideDin_FilterCountT()
        req.portNum = request.port_num
        req.desiredCount = request.desired_count

        res = zenoh_client.query_one(
            f"{robot_model}/save_side_din_filter",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]

    # ==========================================================================
    # 3. Flange IO Control
    # ==========================================================================   
    
    async def call_flange_power(self, robot_model: str, request: Request_Flange_PowerPD):
        req = Request_Flange_PowerT()
        req.desiredVoltage = request.desired_voltage

        res = zenoh_client.query_one(
            f"{robot_model}/call_flange_power",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]


    async def call_flange_dout(self, robot_model: str, request: Request_Flange_Digital_OutPD):
        req = Request_Flange_Digital_OutT()
        req.portNum = request.port_num
        req.desiredOut = request.desired_out

        res = zenoh_client.query_one(
            f"{robot_model}/call_flange_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )
        return res["dict_payload"]