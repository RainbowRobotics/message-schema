"""
[AMR Map 모델]
"""
from typing import Any
import uuid
from enum import Enum
from pydantic import BaseModel
from dataclasses import (
    dataclass,
    field,
)

from rb_utils.service_exception import ServiceException

class AmrMapCommandEnum(str, Enum):
    """
    [AMR Map 명령]
    """
    GET_MAP_LIST = "getMapList"
    DELETE_MAP = "deleteMap"
    GET_MAP_CURRENT = "getMapCurrent"
    GET_MAP_FILE = "getMapFile"
    GET_MAP_CLOUD = "getMapCloud"
    SET_MAP_CLOUD = "setMapCloud"
    GET_MAP_TOPOLOGY = "getMapTopology"
    SET_MAP_TOPOLOGY = "setMapTopology"
    MAP_LOAD = "mapLoad"
    MAP_TOPO_LOAD = "mapTopoLoad"
    MAPPING_START = "mappingStart"
    MAPPING_STOP = "mappingStop"
    MAPPING_SAVE = "mappingSave"

class MapFileInfo(BaseModel):
    """ 맵 파일 정보 """
    fileName: str
    createdAt: str
    updatedAt: str
    fileType: str
    fileSize: float

class MapFile(BaseModel):
    """ 맵 정보 """
    mapName: str
    createdAt: str
    updatedAt: str
    mapType: str | None = None
    cloudInfo: list[MapFileInfo] | None = None
    topoInfo: list[MapFileInfo] | None = None

class Link(BaseModel):
    """ 토폴로지 링크 정보 """
    id: str
    dir: str
    method: str
    speed: float
    safety_field: int

class NodePose(BaseModel):
    """ 토폴로지 노드 위치 정보 """
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

class NodeSize(BaseModel):
    """ 토폴로지 노드 크기 정보 """
    x: float
    y: float
    z: float

class Node(BaseModel):
    """ 토폴로지 노드 정보 """
    context: str | None = None
    id: str
    links: list[Link]
    name: str
    pose: NodePose
    role: str
    size: NodeSize
    type: str

@dataclass
class MapModel:
    """
    [AMR Map 모델]
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    robot_model: str | None = None
    command: AmrMapCommandEnum | None = None

    map_name: str | None = None
    file_name: str | None = None

    # Map List
    map_list: list[MapFile] | None = None

    # Cloud
    cloud_data: list[list[float]] | list[float] | None = None
    cloud_column_count: int | None = None
    cloud_row_count: int | None = None

    # Topology
    topo_data: list[Node] | None = None

    # Result
    result: str | None = None
    message: str | None = None

    def __init__(self, robot_model: str):
        self.id = str(uuid.uuid4())
        self.robot_model = robot_model

    def get_map_list(self):
        """ map list 조회 """
        self.command = AmrMapCommandEnum.GET_MAP_LIST

    def delete_map(self, map_name: str):
        """ map 삭제 """
        self.command = AmrMapCommandEnum.DELETE_MAP
        self.map_name = map_name

    def get_map_current(self):
        """ map current 조회 """
        self.command = AmrMapCommandEnum.GET_MAP_CURRENT

    def get_map_file(self, map_name: str, file_name: str):
        """ map file 조회 """
        self.command = AmrMapCommandEnum.GET_MAP_FILE
        self.map_name = map_name
        self.file_name = file_name

    def get_map_cloud(self, map_name: str, file_name: str):
        """ map cloud 조회 """
        self.command = AmrMapCommandEnum.GET_MAP_CLOUD
        self.map_name = map_name
        self.file_name = file_name

    def set_map_cloud(self, map_name: str, file_name: str, data: list[list[float]]):
        """ map cloud 설정 """
        self.command = AmrMapCommandEnum.SET_MAP_CLOUD
        self.map_name = map_name
        self.file_name = file_name

        self.cloud_data = []
        self.cloud_column_count = len(data[0]) if isinstance(data, list) else 0
        self.cloud_row_count = len(data) if isinstance(data, list) else 0
        for row in data:
            if len(row) != self.cloud_column_count:
                raise ServiceException("data의 열 개수가 일정하지 않습니다", status_code=400)
            for item in row:
                self.cloud_data.append(item)

    def get_map_topology(self, map_name: str, file_name: str):
        """ map topology 조회 """
        self.command = AmrMapCommandEnum.GET_MAP_TOPOLOGY
        self.map_name = map_name
        self.file_name = file_name

    def set_map_topology(self, map_name: str, file_name: str, data: list[Node]):
        """ map topology 설정 """
        self.command = AmrMapCommandEnum.SET_MAP_TOPOLOGY
        self.map_name = map_name
        self.file_name = file_name
        self.topo_data = data

    def map_load(self, map_name: str):
        """ map 로드 """
        self.command = AmrMapCommandEnum.MAP_LOAD
        self.map_name = map_name

    def map_topo_load(self):
        """ map topology 로드 """
        self.command = AmrMapCommandEnum.MAP_TOPO_LOAD

    def mapping_start(self):
        """ mapping 시작 """
        self.command = AmrMapCommandEnum.MAPPING_START

    def mapping_stop(self):
        """ mapping 중지 """
        self.command = AmrMapCommandEnum.MAPPING_STOP

    def mapping_save(self, map_name: str):
        """ mapping 저장 """
        self.command = AmrMapCommandEnum.MAPPING_SAVE
        self.map_name = map_name

    def check_variables(self) -> None:
        """ 변수 검사 """
        if self.command == AmrMapCommandEnum.GET_MAP_LIST:
            pass
        elif self.command == AmrMapCommandEnum.DELETE_MAP:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.GET_MAP_CURRENT:
            pass
        elif self.command == AmrMapCommandEnum.GET_MAP_CLOUD:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
            if self.file_name is None or self.file_name == "":
                raise ServiceException("file_name 값이 비어있습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.SET_MAP_CLOUD:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
            if self.file_name is None or self.file_name == "":
                raise ServiceException("file_name 값이 비어있습니다", status_code=400)
            if self.cloud_data is None or len(self.cloud_data) == 0:
                raise ServiceException("cloud_data 값이 비어있습니다", status_code=400)
            if self.cloud_column_count is None or self.cloud_column_count <= 0:
                raise ServiceException("cloud_column_count 값이 비어있거나 0보다 작습니다", status_code=400)
            if self.cloud_row_count is None or self.cloud_row_count <= 0:
                raise ServiceException("cloud_row_count 값이 비어있거나 0보다 작습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.GET_MAP_TOPOLOGY:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
            if self.file_name is None or self.file_name == "":
                raise ServiceException("file_name 값이 비어있습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.SET_MAP_TOPOLOGY:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
            if self.file_name is None or self.file_name == "":
                raise ServiceException("file_name 값이 비어있습니다", status_code=400)
            if self.topo_data is None:
                raise ServiceException("topo_data 값이 비어있습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.MAP_LOAD:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
        elif self.command == AmrMapCommandEnum.MAP_TOPO_LOAD:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_START:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_STOP:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_SAVE:
            if self.map_name is None or self.map_name == "":
                raise ServiceException("map_name 값이 비어있습니다", status_code=400)
        else:
            raise ServiceException("알 수 없는 command 값입니다. ({self.command})", status_code=400)

    def parse_cloud_data(self, data: list[float], column_count: int, row_count: int):
        """ cloud data 파싱 """
        self.cloud_data = []
        self.cloud_column_count = column_count
        self.cloud_row_count = row_count
        for i in range(row_count):
            self.cloud_data.append([data[i * column_count + j] for j in range(column_count)])

    def to_dict(self) -> dict[str, Any]:
        """ dict 변환 """
        d = {}
        if self.command == AmrMapCommandEnum.GET_MAP_LIST:
            d["list"] = self.map_list
        elif self.command == AmrMapCommandEnum.DELETE_MAP:
            d["mapName"] = self.map_name
        elif self.command == AmrMapCommandEnum.GET_MAP_CURRENT:
            pass
        elif self.command == AmrMapCommandEnum.GET_MAP_CLOUD:
            d["mapName"] = self.map_name
            d["fileName"] = self.file_name
            d["data"] = self.cloud_data
            d["columnCount"] = self.cloud_column_count
            d["rowCount"] = self.cloud_row_count
        elif self.command == AmrMapCommandEnum.SET_MAP_CLOUD:
            d["mapName"] = self.map_name
            d["fileName"] = self.file_name
        elif self.command == AmrMapCommandEnum.GET_MAP_TOPOLOGY:
            d["mapName"] = self.map_name
            d["fileName"] = self.file_name
            d["data"] = self.topo_data
        elif self.command == AmrMapCommandEnum.MAP_LOAD:
            d["mapName"] = self.map_name
        elif self.command == AmrMapCommandEnum.MAP_TOPO_LOAD:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_START:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_STOP:
            pass
        elif self.command == AmrMapCommandEnum.MAPPING_SAVE:
            d["mapName"] = self.map_name

        d["result"] = self.result
        d["message"] = self.message
        return d
