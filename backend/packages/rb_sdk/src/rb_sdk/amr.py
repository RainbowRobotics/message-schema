

from .amr_sdk.amr_control import RBAmrControlSDK
from .amr_sdk.amr_localization import RBAmrLocalizationSDK
from .amr_sdk.amr_map import RBAmrMapSDK
from .amr_sdk.amr_move import RBAmrMoveSDK
from .amr_sdk.amr_setting import RBAmrSettingSDK
from .base import RBBaseSDK
from .amr_sdk.amr_file import RBAmrFileSDK


class RBAmrSDK(RBBaseSDK):
    """Rainbow Robotics AMR SDK"""

    move: RBAmrMoveSDK
    """AMR 이동 관련 SDK 집합"""

    control: RBAmrControlSDK
    """AMR 제어 관련 SDK 집합"""

    localization: RBAmrLocalizationSDK
    """AMR 위치 추정 관련 SDK 집합"""

    map: RBAmrMapSDK
    """AMR 맵 관련 SDK 집합"""

    setting: RBAmrSettingSDK
    """AMR 설정 관련 SDK 집합"""

    file: RBAmrFileSDK
    """AMR 파일 관련 SDK 집합"""

    def __init__(self):
        super().__init__()

        self.move = RBAmrMoveSDK()
        self.control = RBAmrControlSDK()
        self.localization = RBAmrLocalizationSDK()
        self.map = RBAmrMapSDK()
        self.setting = RBAmrSettingSDK()
        self.file = RBAmrFileSDK()
