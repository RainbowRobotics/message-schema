from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Current_Version import Response_Current_VersionT
from rb_flat_buffers.SLAMNAV.Response_Update import Response_UpdateT


@runtime_checkable
class SlamnavUpdatePort(Protocol):
    @abstractmethod
    async def update(self, req_id: str, branch: str, version: str) -> Response_UpdateT: ...
    @abstractmethod
    async def current_version(self, req_id: str) -> Response_Current_VersionT: ...
