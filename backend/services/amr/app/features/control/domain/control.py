
# === Enums ==========================================================
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum
from typing import Any

from rb_sdk.amr_sdk.amr_control import SafetyFlag
from rb_utils.service_exception import ServiceException
from app.schema.amr import AmrResponseStatusEnum
from app.features.control.control_schema import (
    RequestControlChargeTriggerPD,
    RequestControlDetectMarkerPD,
    RequestControlDockPD,
    RequestControlLedModePD,
    RequestControlMotorModePD,
    RequestControlSetObsBoxPD,
    RequestSetSafetyFieldPD,
    RequestSetSafetyFlagPD,
    RequestSetSafetyIoPD,
    ResponseControlChargeTriggerPD,
    ResponseControlDetectMarkerPD,
    ResponseControlDockPD,
    ResponseControlGetObsBoxPD,
    ResponseControlLedModePD,
    ResponseControlSetObsBoxPD,
    ResponseGetSafetyFieldPD,
    ResponseGetSafetyFlagPD,
    ResponseGetSafetyIoPD,
    ResponseSetSafetyFieldPD,
    ResponseSetSafetyFlagPD,
    ResponseSetSafetyIoPD,
)

class AmrLedColorEnum(str, Enum):
    """
    [AMR LED 색상]
    """
    NONE = "none"
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    YELLOW = "yellow"
    WHITE = "white"
    MAGENTA = "magenta"
    RED_BLINK = "red_blink"
    GREEN_BLINK = "green_blink"
    BLUE_BLINK = "blue_blink"
    YELLOW_BLINK = "yellow_blink"
    WHITE_BLINK = "white_blink"
    MAGENTA_BLINK = "magenta_blink"
    UNKNOWN = "unknown"

class AmrControlCommandEnum(str, Enum):
    """
    [AMR 제어 명령]
    """
    CONTROL_DOCK = "dock"
    CONTROL_UNDOCK = "undock"
    CONTROL_DOCK_STOP = "dockStop"
    CONTROL_CHARGE_TRIGGER = "chargeTrigger"
    CONTROL_GET_SAFETY_FIELD = "getSafetyField"
    CONTROL_SET_SAFETY_FIELD = "setSafetyField"
    CONTROL_GET_SAFETY_FLAG = "getSafetyFlag"
    CONTROL_SET_SAFETY_FLAG = "setSafetyFlag"
    CONTROL_LED = "led"
    CONTROL_MOTOR = "motor"
    CONTROL_GET_SAFETY_IO = "getSafetyIo"
    CONTROL_SET_SAFETY_IO = "setSafetyIo"
    CONTROL_GET_OBS_BOX = "getObsBox"
    CONTROL_SET_OBS_BOX = "setObsBox"
    CONTROL_DETECT = "detectMarker"


# === Model ==========================================================
@dataclass
class ControlModel:
    """
    [AMR 이동 모델]
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    robot_model: str | None = None
    command: AmrControlCommandEnum | None = None
    status: AmrResponseStatusEnum = field(default=AmrResponseStatusEnum.PENDING)
    created_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    update_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    result: str | None = None
    message: str | None = None

    control: bool | None = None
    frequency: int | None = None

    # controlLed
    color: AmrLedColorEnum | None = None

    # controlSafety
    safety_field: int | None = None
    safety_flag: list[SafetyFlag] | None = None

    # controlSafetyIo
    mcu0_dio: list[bool] | None = None
    mcu1_dio: list[bool] | None = None
    mcu0_din: list[bool] | None = None
    mcu1_din: list[bool] | None = None

    # controlObsBox
    min_x: float | None = None
    min_y: float | None = None
    min_z: float | None = None
    max_x: float | None = None
    max_y: float | None = None
    max_z: float | None = None
    map_range: float | None = None

    # ExternalAccessory
    foot_position: str | None = None

    # controlDetect
    camera_number: int | None = None
    camera_serial: str | None = None
    marker_size: int | None = None

    def set_robot_model(self, robot_model: str):
        """
        """
        self.robot_model = robot_model
        self.update_at = datetime.now(UTC)


    def control_dock(self, req: RequestControlDockPD):
        """
        """
        self.command = req.command
        self.update_at = datetime.now(UTC)

    def set_control_led(self, req: RequestControlLedModePD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_LED
        self.control = req.control
        self.color = req.color
        self.update_at = datetime.now(UTC)

    def control_motor(self, req: RequestControlMotorModePD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_MOTOR
        self.control = req.control
        self.update_at = datetime.now(UTC)

    def get_control_safety_field(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD
        self.update_at = datetime.now(UTC)

    def set_control_safety_field(self, req: RequestSetSafetyFieldPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_SAFETY_FIELD
        self.safety_field = req.safety_field
        self.update_at = datetime.now(UTC)

    def get_control_safety_flag(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_SAFETY_FLAG
        self.update_at = datetime.now(UTC)

    def set_control_safety_flag(self, req: RequestSetSafetyFlagPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG
        self.safety_flag = req.safetyFlag
        self.update_at = datetime.now(UTC)

    def get_control_safety_io(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_SAFETY_IO
        self.update_at = datetime.now(UTC)

    def set_control_safety_io(self, req: RequestSetSafetyIoPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_SAFETY_IO
        self.mcu0_dio = req.mcu0Din
        self.mcu1_dio = req.mcu1Din
        self.update_at = datetime.now(UTC)

    def get_control_obs_box(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_OBS_BOX
        self.update_at = datetime.now(UTC)

    def set_control_obs_box(self, req: RequestControlSetObsBoxPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_OBS_BOX
        self.min_x = req.minX
        self.min_y = req.minY
        self.min_z = req.minZ
        self.max_x = req.maxX
        self.max_y = req.maxY
        self.max_z = req.maxZ
        self.map_range = req.mapRange
        self.update_at = datetime.now(UTC)


    def set_control_detect(self, req: RequestControlDetectMarkerPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_DETECT
        self.camera_number = req.cameraNumber
        self.camera_serial = req.cameraSerial
        self.marker_size = req.markerSize
        self.update_at = datetime.now(UTC)

    def set_control_charge_trigger(self, req: RequestControlChargeTriggerPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_CHARGE_TRIGGER
        self.control = req.control
        self.update_at = datetime.now(UTC)


    def parse_status(self, status: str) -> AmrResponseStatusEnum:
        """
        """
        try:
            return AmrResponseStatusEnum(status)
        except ValueError:
            return AmrResponseStatusEnum.UNKNOWN

    def result_change(self, result: str) -> None:
        """
        """
        self.result = result
        self.update_at = datetime.now(UTC)

    def status_change(self, status: str) -> None:
        """
        """
        self.status = self.parse_status(status)
        self.update_at = datetime.now(UTC)

    def check_variables(self) -> None:
        """
        """
        if self.robot_model is None:
            raise ServiceException("robotModel 값이 비어있습니다", status_code=400)

        if self.command == AmrControlCommandEnum.CONTROL_DOCK or self.command == AmrControlCommandEnum.CONTROL_UNDOCK or self.command == AmrControlCommandEnum.CONTROL_DOCK_STOP:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_LED:
            if self.control is None:
                raise ServiceException("control 값이 비어있습니다", status_code=400)
            if self.color is None:
                raise ServiceException("color 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_MOTOR:
            if self.control is None:
                raise ServiceException("control 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FIELD:
            if self.safety_field is None:
                raise ServiceException("safetyField 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FLAG:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG:
            if self.safety_flag is None:
                raise ServiceException("safetyFlag 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_IO:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_IO:
            if self.mcu0_dio is None:
                raise ServiceException("mcu0Dio 값이 비어있습니다", status_code=400)
            if self.mcu1_dio is None:
                raise ServiceException("mcu1Dio 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_OBS_BOX:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_OBS_BOX:
            if self.min_x is None:
                raise ServiceException("minX 값이 비어있습니다", status_code=400)
            if self.min_y is None:
                raise ServiceException("minY 값이 비어있습니다", status_code=400)
            if self.min_z is None:
                raise ServiceException("minZ 값이 비어있습니다", status_code=400)
            if self.max_x is None:
                raise ServiceException("maxX 값이 비어있습니다", status_code=400)
            if self.max_y is None:
                raise ServiceException("maxY 값이 비어있습니다", status_code=400)
            if self.max_z is None:
                raise ServiceException("maxZ 값이 비어있습니다", status_code=400)
            if self.map_range is None:
                raise ServiceException("map_range 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_DETECT or self.command == AmrControlCommandEnum.CONTROL_CHARGE_TRIGGER:
            pass
        else:
            raise ServiceException(f"알 수 없는 command 값입니다. ({self.command})", status_code=400)

    def to_dict(self) -> dict[str, Any]:
        """
        """
        d = {}
        d["id"] = self.id
        d["command"] = self.command
        d["status"] = self.status
        d["createdAt"] = self.created_at
        d["updateAt"] = self.update_at
        d["result"] = self.result
        d["message"] = self.message
        if self.command == AmrControlCommandEnum.CONTROL_DOCK or self.command == AmrControlCommandEnum.CONTROL_UNDOCK or self.command == AmrControlCommandEnum.CONTROL_DOCK_STOP:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_LED:
            d["control"] = self.control
            d["color"] = self.color
        elif self.command == AmrControlCommandEnum.CONTROL_MOTOR:
            d["control"] = self.control
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FIELD:
            d["safetyField"] = self.safety_field
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FLAG:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG:
            d["safetyFlag"] = self.safety_flag
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_IO:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_IO:
            d["mcu0Dio"] = self.mcu0_dio
            d["mcu1Dio"] = self.mcu1_dio
        elif self.command == AmrControlCommandEnum.CONTROL_GET_OBS_BOX:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_OBS_BOX:
            d["minX"] = self.min_x
            d["minY"] = self.min_y
            d["minZ"] = self.min_z
            d["maxX"] = self.max_x
            d["maxY"] = self.max_y
            d["maxZ"] = self.max_z
            d["mapRange"] = self.map_range
        elif self.command == AmrControlCommandEnum.CONTROL_DETECT or self.command == AmrControlCommandEnum.CONTROL_CHARGE_TRIGGER:
            pass

        return self.__dict__
