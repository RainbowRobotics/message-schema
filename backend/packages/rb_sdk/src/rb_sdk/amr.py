

from .amr_sdk.amr_control import RBAmrControlSDK
from .amr_sdk.amr_localization import RBAmrLocalizationSDK
from .amr_sdk.amr_map import RBAmrMapSDK
from .amr_sdk.amr_move import RBAmrMoveSDK
from .amr_sdk.amr_setting import RBAmrSettingSDK
from .base import RBBaseSDK


class RBAmrSDK(RBBaseSDK):
    """Rainbow Robotics AMR SDK"""
    def __init__(self):
        super().__init__()
        self.move = RBAmrMoveSDK(client=self.zenoh_client)
        self.control = RBAmrControlSDK(client=self.zenoh_client)
        self.localization = RBAmrLocalizationSDK(client=self.zenoh_client)
        self.map = RBAmrMapSDK(client=self.zenoh_client)
        self.setting = RBAmrSettingSDK(client=self.zenoh_client)
