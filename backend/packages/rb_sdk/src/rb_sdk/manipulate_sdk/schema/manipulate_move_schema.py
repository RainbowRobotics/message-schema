from typing import TypedDict


class MoveInputTargetSchema(TypedDict):
    """Move Input Target 딕셔너리"""
    tar_values: list[float]
    tar_frame: int
    tar_unit: int

class MoveInputSpeedSchema(TypedDict):
    """Move Input Speed 딕셔너리"""
    spd_mode: int
    spd_vel_para: float
    spd_acc_para: float
