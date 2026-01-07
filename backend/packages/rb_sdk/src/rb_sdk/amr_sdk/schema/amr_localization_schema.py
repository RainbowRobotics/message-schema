from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Localization_AutoInit import Response_Localization_AutoInitT
from rb_flat_buffers.SLAMNAV.Response_Localization_Init import Response_Localization_InitT
from rb_flat_buffers.SLAMNAV.Response_Localization_RandomInit import (
    Response_Localization_RandomInitT,
)
from rb_flat_buffers.SLAMNAV.Response_Localization_SemiAutoInit import (
    Response_Localization_SemiAutoInitT,
)
from rb_flat_buffers.SLAMNAV.Response_Localization_Start import Response_Localization_StartT
from rb_flat_buffers.SLAMNAV.Response_Localization_Stop import Response_Localization_StopT


@runtime_checkable
class SlamnavLocalizationPort(Protocol):
    @abstractmethod
    async def localization_init(self, req_id: str, x: float, y: float, z: float, rz: float) -> Response_Localization_InitT: ...
    @abstractmethod
    async def localization_semi_auto_init(self, req_id: str) -> Response_Localization_SemiAutoInitT: ...
    @abstractmethod
    async def localization_auto_init(self, req_id: str) -> Response_Localization_AutoInitT: ...
    @abstractmethod
    async def localization_start(self, req_id: str) -> Response_Localization_StartT: ...
    @abstractmethod
    async def localization_stop(self, req_id: str) -> Response_Localization_StopT: ...
    @abstractmethod
    async def localization_random_init(self, req_id: str, random_seed: str) -> Response_Localization_RandomInitT: ...
