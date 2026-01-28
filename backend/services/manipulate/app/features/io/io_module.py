
from rb_flat_buffers.IPC.Request_Flange_Digital_Out import Request_Flange_Digital_OutT
from rb_flat_buffers.IPC.Request_Flange_Power import Request_Flange_PowerT
from rb_flat_buffers.IPC.Request_Save_SideDin_FilterCount import Request_Save_SideDin_FilterCountT
from rb_flat_buffers.IPC.Request_Save_SideDin_SpecialFunc import Request_Save_SideDin_SpecialFuncT
from rb_flat_buffers.IPC.Request_Save_SideDout_SpecialFunc import Request_Save_SideDout_SpecialFuncT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_modules.service import BaseService
from rb_sdk.manipulate import RBManipulateSDK
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from .io_schema import (
    Request_Flange_Digital_OutPD,
    Request_Flange_PowerPD,
    Request_Multiple_SideAoutPD,
    Request_Multiple_SideDoutBitcombinationPD,
    Request_Multiple_SideDoutPD,
    Request_Multiple_SideDoutPulsePD,
    Request_Multiple_SideDoutTogglePD,
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
rb_manipulate_sdk = RBManipulateSDK()

class IoService(BaseService):
    def __init__(self):
        pass

    # ==========================================================================
    # 1. Side IO Control
    # ==========================================================================

    def call_side_dout(self, robot_model: str, request: Request_SideDout_GeneralPD):
        res = rb_manipulate_sdk.io.call_side_dout(
            robot_model=robot_model,
            port_num=request.port_num,
            desired_out=request.desired_out
        )

        return t_to_dict(res)


    def call_side_dout_toggle(self, robot_model: str, request: Request_SideDout_TogglePD):
        res = rb_manipulate_sdk.io.call_side_dout_toggle(
            robot_model=robot_model,
            port_num=request.port_num
        )

        return t_to_dict(res)


    def call_side_dout_bitcombination(self, robot_model: str, request: Request_SideDout_BitcombinationPD):
        res = rb_manipulate_sdk.io.call_side_dout_bitcombination(
            robot_model=robot_model,
            port_start=request.port_start,
            port_end=request.port_end,
            desired_value=request.desired_value,
            direction_option=request.direction_option
        )

        return t_to_dict(res)


    def call_side_dout_pulse(self, robot_model: str, request: Request_SideDout_PulsePD):
        res = rb_manipulate_sdk.io.call_side_dout_pulse(
            robot_model=robot_model,
            port_num=request.port_num,
            block_mode=request.block_mode,
            direction=request.direction,
            time_1=request.time_1,
            time_2=request.time_2,
            time_3=request.time_3
        )

        return t_to_dict(res)


    def call_multiple_side_dout(self, robot_model: str, request: Request_Multiple_SideDoutPD):
        args_dict_list = [item.model_dump() for item in request.side_dout_args]

        res = rb_manipulate_sdk.io.call_multiple_side_dout(
            robot_model=robot_model,
            side_dout_args=args_dict_list
        )

        return t_to_dict(res)


    def call_multiple_side_dout_toggle(self, robot_model: str, request: Request_Multiple_SideDoutTogglePD):
        args_dict_list = [item.model_dump() for item in request.side_dout_args]

        res = rb_manipulate_sdk.io.call_multiple_side_dout_toggle(
            robot_model=robot_model,
            side_dout_args=args_dict_list
        )

        return t_to_dict(res)


    def call_multiple_side_dout_bitcombination(self, robot_model: str, request: Request_Multiple_SideDoutBitcombinationPD):
        args_dict_list = [item.model_dump() for item in request.side_dout_args]

        res = rb_manipulate_sdk.io.call_multiple_side_dout_bitcombination(
            robot_model=robot_model,
            side_dout_args=args_dict_list
        )

        return t_to_dict(res)


    def call_multiple_side_dout_pulse(self, robot_model: str, request: Request_Multiple_SideDoutPulsePD):
        args_dict_list = [item.model_dump() for item in request.side_dout_args]

        res = rb_manipulate_sdk.io.call_multiple_side_dout_pulse(
            robot_model=robot_model,
            side_dout_args=args_dict_list
        )

        return t_to_dict(res)


    def call_side_aout(self, robot_model: str, request: Request_SideAout_GeneralPD):
        res = rb_manipulate_sdk.io.call_side_aout(
            robot_model=robot_model,
            port_num=request.port_num,
            desired_voltage=request.desired_voltage
        )

        return t_to_dict(res)

    def call_multiple_side_aout(self, robot_model: str, request: Request_Multiple_SideAoutPD):
        args_dict_list = [item.model_dump() for item in request.side_aout_args]

        res = rb_manipulate_sdk.io.call_multiple_side_aout(
            robot_model=robot_model,
            side_aout_args=args_dict_list
        )

        return t_to_dict(res)

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
        return t_to_dict(res)["dict_payload"]


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
        return t_to_dict(res)["dict_payload"]


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
        return t_to_dict(res)["dict_payload"]

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
        return t_to_dict(res)["dict_payload"]


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
        return t_to_dict(res)["dict_payload"]
