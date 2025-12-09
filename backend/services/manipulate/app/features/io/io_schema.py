from pydantic import BaseModel


class Request_SideDout_GeneralPD(BaseModel):
    port_num: int
    desired_out: int

class Request_Multiple_SideDoutPD(BaseModel):
    side_dout_args: list[Request_SideDout_GeneralPD]

class Request_SideAout_GeneralPD(BaseModel):
    port_num: int
    desired_voltage: float


class Request_Save_SideDin_SpecialFuncPD(BaseModel):
    port_num: int
    desired_function: int


class Request_Save_SideDout_SpecialFuncPD(BaseModel):
    port_num: int
    desired_function: int


class Request_Save_SideDin_FilterCountPD(BaseModel):
    port_num: int
    desired_count: int


class Request_Flange_PowerPD(BaseModel):
    desired_voltage: int


class Request_Flange_Digital_OutPD(BaseModel):
    port_num: int
    desired_out: int


class Request_SideDout_TogglePD(BaseModel):
    port_num: int

class Request_Multiple_SideDoutTogglePD(BaseModel):
    side_dout_args: list[Request_SideDout_TogglePD]

class Request_SideDout_BitcombinationPD(BaseModel):
    port_start: int
    port_end: int
    desired_value: int
    direction_option: int

class Request_Multiple_SideDoutBitcombinationPD(BaseModel):
    side_dout_args: list[Request_SideDout_BitcombinationPD]

class Request_SideDout_PulsePD(BaseModel):
    port_num: int
    block_mode: int
    direction: int
    time_1: float
    time_2: float
    time_3: float

class Request_Multiple_SideDoutPulsePD(BaseModel):
    side_dout_args: list[Request_SideDout_PulsePD]
