from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Get_Pdu_Param import Response_Get_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Response_Get_Robot_Type import Response_Get_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Response_Get_Sensor_Index import Response_Get_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Response_Get_Sensor_Off import Response_Get_Sensor_OffT
from rb_flat_buffers.SLAMNAV.Response_Set_Pdu_Param import Response_Set_Pdu_ParamT
from rb_flat_buffers.SLAMNAV.Response_Set_Robot_Type import Response_Set_Robot_TypeT
from rb_flat_buffers.SLAMNAV.Response_Set_Sensor_Index import Response_Set_Sensor_IndexT
from rb_flat_buffers.SLAMNAV.Response_Set_Sensor_On import Response_Set_Sensor_OnT
from rb_flat_buffers.SLAMNAV.Sensor_Info import Sensor_InfoT
from rb_flat_buffers.SLAMNAV.Setting_Param import Setting_ParamT


@runtime_checkable
class SlamnavSettingPort(Protocol):
    @abstractmethod
    async def get_robot_type(self, req_id: str) -> Response_Get_Robot_TypeT: ...
    @abstractmethod
    async def set_robot_type(self, req_id: str, robot_type: str) -> Response_Set_Robot_TypeT: ...
    @abstractmethod
    async def get_sensor_index(self, req_id: str, target: str) -> Response_Get_Sensor_IndexT: ...
    @abstractmethod
    async def set_sensor_index(self, req_id: str, target: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_IndexT: ...
    @abstractmethod
    async def set_sensor_on(self, req_id: str, index: list[Sensor_InfoT]) -> Response_Set_Sensor_OnT: ...
    @abstractmethod
    async def get_sensor_off(self, req_id: str, index: list[Sensor_InfoT]) -> Response_Get_Sensor_OffT: ...
    @abstractmethod
    async def get_pdu_param(self, req_id: str) -> Response_Get_Pdu_ParamT: ...
    @abstractmethod
    async def set_pdu_param(self, req_id: str, params: list[Setting_ParamT]) -> Response_Set_Pdu_ParamT: ...
