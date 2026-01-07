from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Response_Map_Cloud import Response_Map_CloudT
from rb_flat_buffers.SLAMNAV.Response_Map_File import Response_Map_FileT
from rb_flat_buffers.SLAMNAV.Response_Map_List import Response_Map_ListT
from rb_flat_buffers.SLAMNAV.Response_Map_Load import Response_Map_LoadT
from rb_flat_buffers.SLAMNAV.Response_Map_Topology import Response_Map_TopologyT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Save import Response_Mapping_SaveT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Start import Response_Mapping_StartT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Stop import Response_Mapping_StopT


@runtime_checkable
class SlamnavMapPort(Protocol):
    @abstractmethod
    async def map_load(self, req_id: str, map_name: str) -> Response_Map_LoadT: ...
    @abstractmethod
    async def map_list(self, req_id: str) -> Response_Map_ListT: ...
    @abstractmethod
    async def map_file(self, req_id: str) -> Response_Map_FileT: ...
    @abstractmethod
    async def map_cloud(self, req_id: str) -> Response_Map_CloudT: ...
    @abstractmethod
    async def map_topology(self, req_id: str) -> Response_Map_TopologyT: ...
    @abstractmethod
    async def mapping_start(self, req_id: str) -> Response_Mapping_StartT: ...
    @abstractmethod
    async def mapping_stop(self, req_id: str) -> Response_Mapping_StopT: ...
    @abstractmethod
    async def mapping_save(self, req_id: str, map_name: str) -> Response_Mapping_SaveT: ...
