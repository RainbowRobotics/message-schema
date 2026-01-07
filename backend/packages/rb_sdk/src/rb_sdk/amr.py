

from .amr_sdk.amr_move import RBAmrMoveSDK
from .base import RBBaseSDK
from .amr_sdk.amr_setting import RBAmrSettingSDK
from .amr_sdk.amr_control import RBAmrControlSDK
from .amr_sdk.amr_localization import RBAmrLocalizationSDK
from .amr_sdk.amr_map import RBAmrMapSDK
from .amr_sdk.amr_setting import RBAmrSettingSDK


class RBAmrSDK(RBBaseSDK):
    """Rainbow Robotics AMR SDK"""
    def __init__(self):
        super().__init__()
        self.move_sdk = RBAmrMoveSDK(client=self.zenoh_client)
        self.control_sdk = RBAmrControlSDK(client=self.zenoh_client)
        self.localization_sdk = RBAmrLocalizationSDK(client=self.zenoh_client)
        self.map_sdk = RBAmrMapSDK(client=self.zenoh_client)
        self.setting_sdk = RBAmrSettingSDK(client=self.zenoh_client)
