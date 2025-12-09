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

class ResponseCamelReturnValue(TypedDict):
    """Response Return Value"""
    returnValue: int
