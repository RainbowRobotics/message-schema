from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Dock import Response_DockT
from rb_flat_buffers.SLAMNAV.Response_DockStop import Response_DockStopT
from rb_flat_buffers.SLAMNAV.Response_Frequency import Response_FrequencyT
from rb_flat_buffers.SLAMNAV.Response_Led import Response_LedT
from rb_flat_buffers.SLAMNAV.Response_Motor import Response_MotorT
from rb_flat_buffers.SLAMNAV.Response_Obs_Box import Response_Obs_BoxT
from rb_flat_buffers.SLAMNAV.Response_Random_Sequence import Response_Random_SequenceT
from rb_flat_buffers.SLAMNAV.Response_Reset_Safety_Flag import Response_Reset_Safety_FlagT
from rb_flat_buffers.SLAMNAV.Response_Safety_Field import Response_Safety_FieldT
from rb_flat_buffers.SLAMNAV.Response_Safety_Io import Response_Safety_IoT
from rb_flat_buffers.SLAMNAV.Response_Undock import Response_UndockT


@runtime_checkable
class SlamnavControlPort(Protocol):
    @abstractmethod
    async def control_frequency(self, req_id: str, target: str, onoff: bool, frequency: int) -> Response_FrequencyT: ...
    @abstractmethod
    async def control_safety_field(self, req_id: str, command: str, safety_field: int) -> Response_Safety_FieldT: ...
    @abstractmethod
    async def control_reset_safety_flag(self, req_id: str, reset_flag: str) -> Response_Reset_Safety_FlagT: ...
    @abstractmethod
    async def control_dock(self, req_id: str) -> Response_DockT: ...
    @abstractmethod
    async def control_undock(self, req_id: str) -> Response_UndockT: ...
    @abstractmethod
    async def control_dock_stop(self, req_id: str) -> Response_DockStopT: ...
    @abstractmethod
    async def control_obs_box(self, req_id: str, command: str, obs_box: int) -> Response_Obs_BoxT: ...
    @abstractmethod
    async def control_safety_io(self, req_id: str, command: str, mcu0_dio: list[bool], mcu1_dio: list[bool]) -> Response_Safety_IoT: ...
    @abstractmethod
    async def control_led(self, req_id: str, onoff: bool, color: str) -> Response_LedT: ...
    @abstractmethod
    async def control_motor(self, req_id: str, onoff: bool) -> Response_MotorT: ...
    @abstractmethod
    async def control_random_sequence(self, req_id: str) -> Response_Random_SequenceT: ...
