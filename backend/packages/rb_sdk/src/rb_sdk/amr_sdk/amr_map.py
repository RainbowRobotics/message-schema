

from ..base import RBBaseSDK


class RBAmrMapSDK(RBBaseSDK):
    """Rainbow Robotics AMR Map SDK"""

    # async def get_map_file(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_FileT:
    #     """
    #     [Map File 전송]
    #     - model: MapRequestModel
    #     - Response_Get_Map_FileT 객체 반환
    #     """
    #     # 1) Request_Get_Map_FileT 객체 생성
    #     req = Request_Get_Map_FileT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     req.file_name = file_name
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/file",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Get_Map_FileT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Map List failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def get_map_list(self, robot_model: str, req_id: str) -> Response_Map_ListT:
    #     """
    #     [Map List 전송]
    #     - model: MapRequestModel
    #     - Response_Map_ListT 객체 반환
    #     """
    #     # 1) Request_Map_ListT 객체 생성
    #     req = Request_Map_ListT()
    #     req.id = req_id
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/list",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Map_ListT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Map Load failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def map_load(self, robot_model: str, req_id: str, map_name: str) -> Response_Map_LoadT:
    #     """
    #     [Map Load 전송]
    #     - model: MapRequestModel
    #     - Response_Map_LoadT 객체 반환
    #     """
    #     # 1) Request_Map_LoadT 객체 생성
    #     req = Request_Map_LoadT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/load",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Map_LoadT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     return result["dict_payload"]

    # async def get_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_CloudT:
    #     """
    #     [Map Cloud 전송]
    #     - model: MapRequestModel
    #     - Response_Map_CloudT 객체 반환
    #     """
    #     # 1) Request_Map_CloudT 객체 생성
    #     req = Request_Get_Map_CloudT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     req.file_name = file_name

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/cloud/get",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Get_Map_CloudT,
    #         flatbuffer_buf_size=125,
    #     )
    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Map Cloud failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def set_map_cloud(self, robot_model: str, req_id: str, map_name: str, file_name: str, size: int, data: list[float]) -> Response_Set_Map_CloudT:
    #     """
    #     [Map Cloud 전송]
    #     - model: MapRequestModel
    #     - Response_Map_CloudT 객체 반환
    #     """

    #     # 1) Request_Map_CloudT 객체 생성
    #     req = Request_Set_Map_CloudT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     req.file_name = file_name
    #     req.size = size
    #     req.data = data

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/cloud/set",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Set_Map_CloudT,
    #         flatbuffer_buf_size=1000,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Map Cloud failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def get_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str) -> Response_Get_Map_TopologyT:
    #     """
    #     [Map Topology 전송]
    #     - model: MapRequestModel
    #     - Response_Map_TopologyT 객체 반환
    #     """
    #     # 1) Request_Map_TopologyT 객체 생성
    #     req = Request_Get_Map_TopologyT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     req.file_name = file_name

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/topology/get",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Get_Map_TopologyT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Map Topology failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def set_map_topology(self, robot_model: str, req_id: str, map_name: str, file_name: str, data: list[NodeT]) -> Response_Set_Map_TopologyT:
    #     """
    #     [Map Topology 전송]
    #     - model: MapRequestModel
    #     - Response_Map_TopologyT 객체 반환
    #     """
    #     # 1) Request_Map_TopologyT 객체 생성
    #     req = Request_Get_Map_TopologyT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     req.file_name = file_name
    #     req.data = data

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/map/topology/set",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Set_Map_TopologyT,
    #         flatbuffer_buf_size=1000,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Mapping Start failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def mapping_start(self, robot_model: str, req_id: str) -> Response_Mapping_StartT:
    #     """
    #     [Mapping Start 전송]
    #     - model: MapRequestModel
    #     - Response_Mapping_StartT 객체 반환
    #     """
    #     # 1) Request_Mapping_StartT 객체 생성
    #     req = Request_Mapping_StartT()
    #     req.id = req_id
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/mapping/start",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Mapping_StartT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     return result["dict_payload"]

    # async def mapping_stop(self, robot_model: str, req_id: str) -> Response_Mapping_StopT:
    #     """
    #     [Mapping Stop 전송]
    #     - model: MapRequestModel
    #     - Response_Mapping_StopT 객체 반환
    #     """
    #     # 1) Request_Mapping_StopT 객체 생성
    #     req = Request_Mapping_StopT()
    #     req.id = req_id
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/mapping/stop",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Mapping_StopT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Mapping Save failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def mapping_save(self, robot_model: str, req_id: str, map_name: str) -> Response_Mapping_SaveT:
    #     """
    #     [Mapping Save 전송]
    #     - model: MapRequestModel
    #     - Response_Mapping_SaveT 객체 반환
    #     """
    #     # 1) Request_Mapping_SaveT 객체 생성
    #     req = Request_Mapping_SaveT()
    #     req.id = req_id
    #     req.map_name = map_name
    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/mapping/save",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=Response_Mapping_SaveT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Mapping Save failed: obj_payload is None")

    #     return result["obj_payload"]
