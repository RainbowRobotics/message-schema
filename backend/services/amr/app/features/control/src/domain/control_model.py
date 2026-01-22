
# === Enums ==========================================================
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum
from typing import Any

from rb_flat_buffers.SLAMNAV.Safety_Flag import Safety_FlagT
from rb_utils.service_exception import ServiceException

from app.features.control.schema.control_api import (
    Request_Control_DetectPD,
    Request_Control_FrequencyPD,
    Request_Control_LEDPD,
    Request_Control_MotorPD,
    Request_Control_ObsBoxPD,
    Request_Control_SafetyFieldPD,
    Request_Control_SafetyFlagPD,
    Request_Control_SafetyIOPD,
)
from app.schema.amr import AmrResponseStatusEnum


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
    CONTROL_FREQUENCY = "controlFrequency"
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
    CONTROL_DETECT = "detect"


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

    # controlOnoff
    target: str | None = None
    onoff: bool | None = None
    frequency: int | None = None

    # controlLed
    color: AmrLedColorEnum | None = None

    # controlSafety
    safety_field: int | None = None
    safety_flags: list[Safety_FlagT] | None = None

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

    def set_robot_model(self, robot_model: str):
        """
        """
        self.robot_model = robot_model
        self.update_at = datetime.now(UTC)

    def control_frequency(self, req: Request_Control_FrequencyPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_FREQUENCY
        self.target = req.target
        self.onoff = req.onoff
        self.frequency = req.frequency
        self.update_at = datetime.now(UTC)

    def control_dock(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_DOCK
        self.update_at = datetime.now(UTC)

    def control_undock(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_UNDOCK
        self.update_at = datetime.now(UTC)

    def control_dock_stop(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_DOCK_STOP
        self.update_at = datetime.now(UTC)

    def control_led(self, req: Request_Control_LEDPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_LED
        self.onoff = req.onoff
        self.color = req.color
        self.update_at = datetime.now(UTC)

    def control_motor(self, req: Request_Control_MotorPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_MOTOR
        self.onoff = req.onoff
        self.update_at = datetime.now(UTC)

    def get_control_safety_field(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD
        self.update_at = datetime.now(UTC)

    def set_control_safety_field(self, req: Request_Control_SafetyFieldPD):
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

    def set_control_safety_flag(self, req: Request_Control_SafetyFlagPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG
        self.safety_flags = req.safety_flags
        self.update_at = datetime.now(UTC)

    def get_control_safety_io(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_GET_SAFETY_IO
        self.update_at = datetime.now(UTC)

    def set_control_safety_io(self, req: Request_Control_SafetyIOPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_SAFETY_IO
        self.mcu0_dio = req.mcu0_dio
        self.mcu1_dio = req.mcu1_dio
        self.update_at = datetime.now(UTC)

    def set_control_obs_box(self, req: Request_Control_ObsBoxPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_SET_OBS_BOX
        self.min_x = req.min_x
        self.min_y = req.min_y
        self.min_z = req.min_z
        self.max_x = req.max_x
        self.max_y = req.max_y
        self.max_z = req.max_z
        self.map_range = req.map_range
        self.update_at = datetime.now(UTC)


    def set_control_detect(self, req: Request_Control_DetectPD):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_DETECT
        self.update_at = datetime.now(UTC)

    def set_control_charge_trigger(self):
        """
        """
        self.command = AmrControlCommandEnum.CONTROL_CHARGE_TRIGGER
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
            raise ServiceException("robot_model 값이 비어있습니다", status_code=400)
        if self.command == AmrControlCommandEnum.CONTROL_FREQUENCY:
            if self.target is None:
                raise ServiceException("target 값이 비어있습니다", status_code=400)
            if self.onoff is None:
                raise ServiceException("onoff 값이 비어있습니다", status_code=400)
            if self.frequency is None:
                raise ServiceException("frequency 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_DOCK or self.command == AmrControlCommandEnum.CONTROL_UNDOCK or self.command == AmrControlCommandEnum.CONTROL_DOCK_STOP:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_LED:
            if self.onoff is None:
                raise ServiceException("onoff 값이 비어있습니다", status_code=400)
            if self.color is None:
                raise ServiceException("color 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_MOTOR:
            if self.onoff is None:
                raise ServiceException("onoff 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FIELD:
            if self.safety_field is None:
                raise ServiceException("safety_field 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FLAG:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG:
            if self.safety_flags is None:
                raise ServiceException("safety_flags 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_IO:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_IO:
            if self.mcu0_dio is None:
                raise ServiceException("mcu0_dio 값이 비어있습니다", status_code=400)
            if self.mcu1_dio is None:
                raise ServiceException("mcu1_dio 값이 비어있습니다", status_code=400)
        elif self.command == AmrControlCommandEnum.CONTROL_GET_OBS_BOX:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_OBS_BOX:
            if self.min_x is None:
                raise ServiceException("min_x 값이 비어있습니다", status_code=400)
            if self.min_y is None:
                raise ServiceException("min_y 값이 비어있습니다", status_code=400)
            if self.min_z is None:
                raise ServiceException("min_z 값이 비어있습니다", status_code=400)
            if self.max_x is None:
                raise ServiceException("max_x 값이 비어있습니다", status_code=400)
            if self.max_y is None:
                raise ServiceException("max_y 값이 비어있습니다", status_code=400)
            if self.max_z is None:
                raise ServiceException("max_z 값이 비어있습니다", status_code=400)
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
        if self.command == AmrControlCommandEnum.CONTROL_FREQUENCY:
            d["target"] = self.target
            d["onoff"] = self.onoff
            d["frequency"] = self.frequency
        elif self.command == AmrControlCommandEnum.CONTROL_DOCK or self.command == AmrControlCommandEnum.CONTROL_UNDOCK or self.command == AmrControlCommandEnum.CONTROL_DOCK_STOP:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_LED:
            d["onoff"] = self.onoff
            d["color"] = self.color
        elif self.command == AmrControlCommandEnum.CONTROL_MOTOR:
            d["onoff"] = self.onoff
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FIELD:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FIELD:
            d["safetyField"] = self.safety_field
        elif self.command == AmrControlCommandEnum.CONTROL_GET_SAFETY_FLAG:
            pass
        elif self.command == AmrControlCommandEnum.CONTROL_SET_SAFETY_FLAG:
            d["safetyFlags"] = self.safety_flags
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
