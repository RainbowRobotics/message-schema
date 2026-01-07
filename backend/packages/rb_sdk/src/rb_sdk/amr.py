
from rb_flat_buffers.SLAMNAV.Request_Move_Circular import Request_Move_CircularT
from rb_flat_buffers.SLAMNAV.Request_Move_Goal import Request_Move_GoalT
from rb_flat_buffers.SLAMNAV.Request_Move_Jog import Request_Move_JogT
from rb_flat_buffers.SLAMNAV.Request_Move_Pause import Request_Move_PauseT
from rb_flat_buffers.SLAMNAV.Request_Move_Resume import Request_Move_ResumeT
from rb_flat_buffers.SLAMNAV.Request_Move_Rotate import Request_Move_RotateT
from rb_flat_buffers.SLAMNAV.Request_Move_Stop import Request_Move_StopT
from rb_flat_buffers.SLAMNAV.Request_Move_Target import Request_Move_TargetT
from rb_flat_buffers.SLAMNAV.Request_Move_XLinear import Request_Move_XLinearT
from rb_flat_buffers.SLAMNAV.Response_Move_Circular import Response_Move_CircularT
from rb_flat_buffers.SLAMNAV.Response_Move_Goal import Response_Move_GoalT
from rb_flat_buffers.SLAMNAV.Response_Move_Pause import Response_Move_PauseT
from rb_flat_buffers.SLAMNAV.Response_Move_Resume import Response_Move_ResumeT
from rb_flat_buffers.SLAMNAV.Response_Move_Rotate import Response_Move_RotateT
from rb_flat_buffers.SLAMNAV.Response_Move_Stop import Response_Move_StopT
from rb_flat_buffers.SLAMNAV.Response_Move_Target import Response_Move_TargetT
from rb_flat_buffers.SLAMNAV.Response_Move_XLinear import Response_Move_XLinearT
from rb_flat_buffers.SLAMNAV.State_Change_Move import State_Change_MoveT
from rb_utils.parser import t_to_dict

from .base import RBBaseSDK
from .amr_sdk.amr_move import RBAmrMoveSDK

class RBAmrSDK(RBBaseSDK):
    """Rainbow Robotics AMR SDK"""
    def __init__(self):
        super().__init__()
        self.move_sdk = RBAmrMoveSDK(client=self.zenoh_client)
