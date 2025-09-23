from pydantic import BaseModel


class Request_SideDout_GeneralPD(BaseModel):
    port_num: int
    desired_out: int


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
