from typing import TypedDict


class MoveInputTargetSchema(TypedDict):
    """Move Input Target 딕셔너리"""
    tar_values: list[float] # 이동 좌표 리스트
    tar_frame: int # -1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame
    tar_unit: int # 0: mm/degree, 1: meter/radian, 2: inch/degree

class MoveInputSpeedSchema(TypedDict):
    """Move Input Speed 딕셔너리"""
    spd_mode: int # 0: %기반 설정, 1: 절대값(물리값)
    spd_vel_para: float # 속도
    spd_acc_para: float # 가속도

class MoveInputTypeSchema(TypedDict):
    """Move Input Blend Type 딕셔너리"""
    pnt_type: int # 0: Percentage, 1: Distance
    pnt_para: float # pnt_type가 0일때는 %, 1일때는 mm
