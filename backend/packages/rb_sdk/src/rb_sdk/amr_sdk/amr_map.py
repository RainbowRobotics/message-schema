

from rb_flat_buffers.SLAMNAV.Node import NodeT
from rb_flat_buffers.SLAMNAV.RequestGetMapCloud import RequestGetMapCloudT
from rb_flat_buffers.SLAMNAV.RequestGetMapTopology import RequestGetMapTopologyT
from rb_flat_buffers.SLAMNAV.RequestMapCurrent import RequestMapCurrentT
from rb_flat_buffers.SLAMNAV.RequestMapDelete import RequestMapDeleteT
from rb_flat_buffers.SLAMNAV.RequestMapList import RequestMapListT
from rb_flat_buffers.SLAMNAV.RequestMapLoad import RequestMapLoadT
from rb_flat_buffers.SLAMNAV.RequestMappingSave import RequestMappingSaveT
from rb_flat_buffers.SLAMNAV.RequestMappingStart import RequestMappingStartT
from rb_flat_buffers.SLAMNAV.RequestMappingStop import RequestMappingStopT
from rb_flat_buffers.SLAMNAV.RequestSetMapCloud import RequestSetMapCloudT
from rb_flat_buffers.SLAMNAV.RequestSetMapTopology import RequestSetMapTopologyT
from rb_flat_buffers.SLAMNAV.RequestTopoLoad import RequestTopoLoadT
from rb_flat_buffers.SLAMNAV.ResponseGetMapCloud import ResponseGetMapCloudT
from rb_flat_buffers.SLAMNAV.ResponseGetMapTopology import ResponseGetMapTopologyT
from rb_flat_buffers.SLAMNAV.ResponseMapCurrent import ResponseMapCurrentT
from rb_flat_buffers.SLAMNAV.ResponseMapDelete import ResponseMapDeleteT
from rb_flat_buffers.SLAMNAV.ResponseMapList import ResponseMapListT
from rb_flat_buffers.SLAMNAV.ResponseMapLoad import ResponseMapLoadT
from rb_flat_buffers.SLAMNAV.ResponseMappingSave import ResponseMappingSaveT
from rb_flat_buffers.SLAMNAV.ResponseMappingStart import ResponseMappingStartT
from rb_flat_buffers.SLAMNAV.ResponseMappingStop import ResponseMappingStopT
from rb_flat_buffers.SLAMNAV.ResponseSetMapCloud import ResponseSetMapCloudT
from rb_flat_buffers.SLAMNAV.ResponseSetMapTopology import ResponseSetMapTopologyT
from rb_flat_buffers.SLAMNAV.ResponseTopoLoad import ResponseTopoLoadT

from ..base import RBBaseSDK


class RBAmrMapSDK(RBBaseSDK):
    """Rainbow Robotics AMR Map SDK"""

    async def get_map_list(self, robot_model: str, req_id: str) -> ResponseMapListT:
        """
        [Map List 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - ResponseMapListT 객체 반환
        """

        # 1) Request_Map_ListT 객체 생성
        req = RequestMapListT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/list",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMapListT,
            flatbuffer_buf_size=2048,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Load failed: obj_payload is None")

        return result["obj_payload"]

    async def delete_map(self, robot_model: str, req_id: str, map_name: str) -> ResponseMapDeleteT:
        """
        [Map Delete 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - ResponseMapDeleteT 객체 반환
        """

        # 1) RequestMapDeleteT 객체 생성
        req = RequestMapDeleteT()
        req.id = req_id
        req.mapName = map_name

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/delete",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMapDeleteT,
            flatbuffer_buf_size=2048,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Delete failed: obj_payload is None")

        return result["obj_payload"]

    async def get_current_map(self, robot_model: str, req_id: str) -> ResponseMapCurrentT:
        """
        [Current Map 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - ResponseMapCurrentT 객체 반환
        """

        # 1) RequestMapDeleteT 객체 생성
        req = RequestMapCurrentT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/current",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMapCurrentT,
            flatbuffer_buf_size=2048,
        )


        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Current failed: obj_payload is None")

        return result["obj_payload"]

    async def get_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> ResponseGetMapCloudT:
        """
        [Map Cloud 조회]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - file_name: 파일 이름
        - ResponseGetMapCloudT 객체 반환
        """

        # 1) Request_Map_CloudT 객체 생성
        req = RequestGetMapCloudT()
        req.id = req_id
        req.mapName = map_name
        req.fileName = file_name

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/getCloud",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetMapCloudT,
            flatbuffer_buf_size=4096,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Cloud failed: obj_payload is None")

        return result["obj_payload"]

    async def set_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str, data: list[float], column_count: int, row_count: int) -> ResponseSetMapCloudT:
        """
        [Map Cloud 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - file_name: 파일 이름
        - data: 클라우드 데이터
        - column_count: 클라우드 데이터 열 개수
        - row_count: 클라우드 데이터 행 개수
        - ResponseSetMapCloudT 객체 반환
        """

        # 1) RequestSetMapCloudT 객체 생성
        req = RequestSetMapCloudT()
        req.id = req_id
        req.mapName = map_name
        req.fileName = file_name
        req.data = data
        req.columnCount = column_count
        req.rowCount = row_count

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/setCloud",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetMapCloudT,
            flatbuffer_buf_size=1000,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Cloud failed: obj_payload is None")

        return result["obj_payload"]

    async def get_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str, pageNo: int | None = None, pageSize: int | None = None, nodeType: str | None = None, searchText: str | None = None, sortOption: str | None = None, sortDirection: str | None = None) -> ResponseGetMapTopologyT:
        """
        [Map Topology 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - file_name: 파일 이름
        - pageNo: 페이지 번호
        - pageSize: 페이지 크기
        - nodeType: 노드 타입
        - searchText: 검색어
        - sortOption: 정렬 옵션
        - sortDirection: 정렬 방향
        - ResponseGetMapTopologyT 객체 반환
        """

        # 1) RequestGetMapTopologyT 객체 생성
        req = RequestGetMapTopologyT()
        req.id = req_id
        req.mapName = map_name
        req.fileName = file_name
        req.pageNo = pageNo if pageNo is not None else 1
        req.pageSize = pageSize if pageSize is not None else 10
        req.nodeType = nodeType if nodeType is not None else ""
        req.searchText = searchText if searchText is not None else ""
        req.sortOption = sortOption if sortOption is not None else ""
        req.sortDirection = sortDirection if sortDirection is not None else ""

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/getTopology",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetMapTopologyT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Map Topology failed: obj_payload is None")

        return result["obj_payload"]

    async def set_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str, data: list[NodeT]) -> ResponseSetMapTopologyT:
        """
        [Map Topology 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - file_name: 파일 이름
        - data: 노드 데이터
        - ResponseSetMapTopologyT 객체 반환
        """
        # 1) RequestSetMapTopologyT 객체 생성
        req = RequestSetMapTopologyT()
        req.id = req_id
        req.mapName = map_name
        req.fileName = file_name
        req.data = data

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/setTopology",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetMapTopologyT,
            flatbuffer_buf_size=1000,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Mapping Start failed: obj_payload is None")

        return result["obj_payload"]

    async def mapping_start(self, robot_model: str, req_id: str) -> ResponseMappingStartT:
        """
        [Mapping Start 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - ResponseMappingStartT 객체 반환
        """

        # 1) Request_Mapping_StartT 객체 생성
        req = RequestMappingStartT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/mapping/start",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMappingStartT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def mapping_stop(self, robot_model: str, req_id: str) -> ResponseMappingStopT:
        """
        [Mapping Stop 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - ResponseMappingStopT 객체 반환
        """
        # 1) Request_Mapping_StopT 객체 생성
        req = RequestMappingStopT()
        req.id = req_id
        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/mapping/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMappingStopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Mapping Save failed: obj_payload is None")

        return result["obj_payload"]

    async def mapping_save(self, robot_model: str, req_id: str, map_name: str) -> ResponseMappingSaveT:
        """
        [Mapping Save 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - ResponseMappingSaveT 객체 반환
        """
        # 1) Request_Mapping_SaveT 객체 생성
        req = RequestMappingSaveT()
        req.id = req_id
        req.mapName = map_name

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/mapping/save",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMappingSaveT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Mapping Save failed: obj_payload is None")

        return result["obj_payload"]

    async def map_load(self, robot_model: str, req_id: str, map_name: str) -> ResponseMapLoadT:
        """
        [Map Load 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - map_name: 맵 이름
        - ResponseMapLoadT 객체 반환
        """

        # 1) RequestMapLoadT 객체 생성
        req = RequestMapLoadT()
        req.id = req_id
        req.mapName = map_name
        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/loadMap",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMapLoadT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]

    async def topo_reload(self, robot_model: str, req_id: str) -> ResponseTopoLoadT:
        """
        [Topo Load 전송]
        - robot_model: 요청을 보낼 로봇 모델
        - req_id: 요청 ID
        - ResponseTopoLoadT 객체 반환
        """

        # 1) RequestTopoLoadT 객체 생성
        req = RequestTopoLoadT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/map/loadTopo",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseTopoLoadT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        return result["dict_payload"]
