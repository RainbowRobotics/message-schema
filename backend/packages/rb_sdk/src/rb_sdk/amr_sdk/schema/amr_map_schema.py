from abc import abstractmethod
from typing import (
    Protocol,
    runtime_checkable,
)

from rb_flat_buffers.SLAMNAV.Node import NodeT
from rb_flat_buffers.SLAMNAV.Response_Get_Map_Cloud import Response_Get_Map_CloudT
from rb_flat_buffers.SLAMNAV.Response_Get_Map_File import Response_Get_Map_FileT
from rb_flat_buffers.SLAMNAV.Response_Get_Map_Topology import Response_Get_Map_TopologyT
from rb_flat_buffers.SLAMNAV.Response_Map_List import Response_Map_ListT
from rb_flat_buffers.SLAMNAV.Response_Map_Load import Response_Map_LoadT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Save import Response_Mapping_SaveT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Start import Response_Mapping_StartT
from rb_flat_buffers.SLAMNAV.Response_Mapping_Stop import Response_Mapping_StopT
from rb_flat_buffers.SLAMNAV.Response_Set_Map_Cloud import Response_Set_Map_CloudT
from rb_flat_buffers.SLAMNAV.Response_Set_Map_Topology import Response_Set_Map_TopologyT


@runtime_checkable
class SlamnavMapPort(Protocol):
    @abstractmethod
    async def map_load(self, robot_model: str, req_id: str, map_name: str) -> Response_Map_LoadT: ...
    @abstractmethod
    async def get_map_list(self, robot_model: str, req_id: str) -> Response_Map_ListT: ...
    @abstractmethod
    async def get_map_file(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_FileT: ...
    @abstractmethod
    async def get_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_CloudT: ...
    @abstractmethod
    async def get_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_TopologyT: ...
    @abstractmethod
    async def set_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str, size: int, data: list[float]) -> Response_Set_Map_CloudT: ...
    @abstractmethod
    async def set_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str, data: list[NodeT]) -> Response_Set_Map_TopologyT: ...
    @abstractmethod
    async def mapping_start(self, robot_model: str, req_id: str) -> Response_Mapping_StartT: ...
    @abstractmethod
    async def mapping_stop(self, robot_model: str, req_id: str) -> Response_Mapping_StopT: ...
    @abstractmethod
    async def mapping_save(self, robot_model: str, req_id: str, map_name: str) -> Response_Mapping_SaveT: ...
