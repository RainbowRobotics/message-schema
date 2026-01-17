""" Rainbow Robotics Manipulate Point SDK """
from typing import Literal

from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK
from .manipulate_config import RBManipulateConfigSDK
from .manipulate_move import RBManipulateMoveSDK
from .schema.manipulate_move_schema import (
    MoveInputSpeedSchema,
    MoveInputTargetSchema,
)


class RBManipulatePointSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Point SDK"""

    def __init__(self):
        super().__init__()

        self.manipulate_move_sdk = RBManipulateMoveSDK()
        self.manipulate_config_sdk = RBManipulateConfigSDK()

    def set_point(
        self,
        *,
        robot_model: str,
        move_type: Literal["J", "L", "JB", "LB"],
        input_method: int = 0,
        tar_frame_reference_point: list[int | float] | None = None,
        pnt_para: int | None = None,
        pnt_type: int | None = None,
        tar_frame: int,
        tar_unit: int,
        tar_values: list[float],
        spd_mode: int,
        spd_vel_para: float,
        spd_acc_para: float,
        tcp_num: int = -1,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT | None:
        """
        협동로봇 이동할 point 설정

        Args:
            robot_model: 로봇 모델명
            move_type: 이동 타입 (J: Joint Space, L: Linear Space, JB: Joint Block, LB: Linear Block)
            input_method: 입력 방식 (0: 직접 입력, 1: 상대 좌표 입력)
            tar_frame_reference_point: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            pnt_para: 포인트타입 (0: 직선포인트, 1: %기반 블랜드포인트, 2: 거리기반 블랜드포인트)
            pnt_type: 블랜드 파라미터 (%기반 블랜드포인트나 거리기반 블랜드포인트을 위한 파라미터)
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            tar_values: 이동 좌표 리스트
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            tcp_num: TCP 번호 (-1: 기본값)
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        if tcp_num != -1:
            self.manipulate_config_sdk.set_toolist_num(robot_model=robot_model, tool_num=tcp_num)

        target = MoveInputTargetSchema(
            tar_values=tar_values,
            tar_frame=tar_frame,
            tar_unit=tar_unit,
        )
        speed = MoveInputSpeedSchema(
            spd_mode=spd_mode,
            spd_vel_para=spd_vel_para,
            spd_acc_para=spd_acc_para,
        )

        if input_method == 1:
            if tar_frame_reference_point is None:
                raise RuntimeError("tar_frame_reference_point is required for relative move")

            relative_value = MoveInputTargetSchema(
                tar_values=tar_values,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
            )
            reference_value = MoveInputTargetSchema(
                tar_values=tar_frame_reference_point,
                tar_frame=0,
                tar_unit=0,
            )
            speed = MoveInputSpeedSchema(
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
            )

            return self.manipulate_move_sdk.call_relative_move(
                robot_model=robot_model,
                relative_value=relative_value,
                reference_value=reference_value,
                move_type=1 if move_type.startswith("L") else 0,
                speed=speed,
                flow_manager_args=flow_manager_args,
            )

        elif move_type == "J":
            return self.manipulate_move_sdk.call_move_j(
                robot_model=robot_model,
                target=target,
                speed=speed,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "L":
            return self.manipulate_move_sdk.call_move_l(
                robot_model=robot_model,
                target=target,
                speed=speed,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "JB":
            if pnt_para is None or pnt_type is None:
                raise ValueError("pnt_para and pnt_type are required for JB move")

            return self.manipulate_move_sdk.call_move_jb(
                robot_model=robot_model,
                pnt_para=pnt_para,
                pnt_type=pnt_type,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                tar_values=tar_values,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "LB":
            if pnt_para is None or pnt_type is None:
                raise ValueError("pnt_para and pnt_type are required for LB move")

            return self.manipulate_move_sdk.call_move_lb(
                robot_model=robot_model,
                pnt_para=pnt_para,
                pnt_type=pnt_type,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                tar_values=tar_values,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                flow_manager_args=flow_manager_args,
            )
