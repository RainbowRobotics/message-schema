"""Manipulate SDK 스키마"""
from typing import TypedDict


class SideDoutArg(TypedDict):
    """Side Digital Out 호출 인자"""
    port_num: int
    desired_out: bool

class SideDoutToggleArg(TypedDict):
    """Side Digital Out Toggle 호출 인자"""
    port_num: int

class SideDoutBitcombinationArg(TypedDict):
    """Side Digital Out Bitcombination 호출 인자"""
    port_start: int
    port_end: int
    desired_value: int
    direction_option: int

class SideDoutPulseArg(TypedDict):
    """Side Digital Out Pulse 호출 인자"""
    port_num: int
    block_mode: int
    direction: int
    time_1: int
    time_2: int
    time_3: int

class SideAoutArg(TypedDict):
    """Side Analog Out 호출 인자"""
    port_num: int
    desired_voltage: float

class MultipleSideAoutArg(TypedDict):
    """Multiple Side Analog Out 호출 인자"""
    side_aout_args: list[SideAoutArg]

class ResponseCamelReturnValue(TypedDict):
    """Response Return Value"""
    returnValue: int

class MoveInputTargetSchema(TypedDict):
    """Move Input Target 딕셔너리"""
    tar_values: list[float]
    tar_frame: int
    tar_unit: int

class ResponseGetRelativeValueSchema(TypedDict):
    """Response Get Relative Value 딕셔너리"""
    calculated_result: int
    calculated_value: MoveInputTargetSchema
