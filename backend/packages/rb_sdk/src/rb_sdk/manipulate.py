""" Rainbow Robotics Manipulate Total SDK """

from .base import RBBaseSDK
from .manipulate_sdk.manipulate_config import RBManipulateConfigSDK
from .manipulate_sdk.manipulate_get_data import RBManipulateGetDataSDK
from .manipulate_sdk.manipulate_io import RBManipulateIOSDK
from .manipulate_sdk.manipulate_move import RBManipulateMoveSDK
from .manipulate_sdk.manipulate_point import RBManipulatePointSDK
from .manipulate_sdk.manipulate_program import RBManipulateProgramSDK


class RBManipulateSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Total SDK"""

    program: RBManipulateProgramSDK
    """프로그램 관련 SDK 집합"""

    move: RBManipulateMoveSDK
    """모션/이동 관련 SDK 집합"""

    config: RBManipulateConfigSDK
    """환경설정/파라미터 관련 SDK 집합"""

    io: RBManipulateIOSDK
    """IO 제어 SDK 집합"""

    get_data: RBManipulateGetDataSDK
    """상태/데이터 조회 SDK 집합"""

    point: RBManipulatePointSDK
    """포인트/좌표 관련 SDK 집합"""

    def __init__(self):
        super().__init__()

        self.program = RBManipulateProgramSDK()
        self.move = RBManipulateMoveSDK()
        self.config = RBManipulateConfigSDK()
        self.io = RBManipulateIOSDK()
        self.get_data = RBManipulateGetDataSDK()
        self.point = RBManipulatePointSDK()
