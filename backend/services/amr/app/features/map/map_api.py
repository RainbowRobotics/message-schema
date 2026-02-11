
from fastapi import APIRouter
from fastapi.responses import StreamingResponse

from .map_schema import (
    RequestSetMapCloudPD,
    RequestSetMapTopologyPD,
    ResponseGetMapCloudPD,
    ResponseGetMapTopologyPD,
    ResponseMapListPD,
    ResponseSetMapCloudPD,
    ResponseSetMapTopologyPD,
)
from .map_service import AmrMapService

amr_map_router = APIRouter(
    tags=["AMR 맵"],
    prefix="",
)

amr_map_service = AmrMapService()

@amr_map_router.get("/{robot_model}/map/list", summary="맵 목록 조회", description="맵 목록 조회")
async def get_map_list(robot_model: str) -> ResponseMapListPD:
    return await amr_map_service.get_map_list(robot_model)

# @amr_map_router.delete("/{robot_model}/map/delete", summary="맵 삭제", description="맵 삭제")
# async def delete_map(robot_model: str, request: RequestMapDeletePD) -> ResponseMapDeletePD:
#     return await amr_map_service.delete_map(robot_model, request)

# @amr_map_router.get("/{robot_model}/map/current", summary="현재 맵 조회", description="현재 맵 조회")
# async def get_current_map(robot_model: str) -> ResponseMapCurrentPD:
#     return await amr_map_service.get_current_map(robot_model)

@amr_map_router.get("/{robot_model}/map/getCloud", summary="맵 클라우드 조회", description="맵 클라우드 조회")
async def get_map_cloud(robot_model: str, mapName: str, fileName: str) -> ResponseGetMapCloudPD:
    return await amr_map_service.get_map_cloud(robot_model, mapName, fileName)

@amr_map_router.post("/{robot_model}/map/setCloud", summary="맵 클라우드 설정", description="맵 클라우드 설정")
async def set_map_cloud(robot_model: str, request: RequestSetMapCloudPD) -> ResponseSetMapCloudPD:
    return await amr_map_service.set_map_cloud(robot_model, request)

@amr_map_router.get("/{robot_model}/map/getTopology", summary="맵 토폴로지 조회", description="맵 토폴로지 조회")
async def get_map_topology(robot_model: str, mapName: str, fileName: str, pageNo: int | None = None, pageSize: int | None = None, nodeType: str | None = None, searchText: str | None = None, sortOption: str | None = None, sortDirection: str | None = None) -> ResponseGetMapTopologyPD:
    return await amr_map_service.get_map_topology(robot_model, mapName, fileName, pageNo=pageNo, pageSize=pageSize, nodeType=nodeType, searchText=searchText, sortOption=sortOption, sortDirection=sortDirection)

@amr_map_router.post("/{robot_model}/map/setTopology", summary="맵 토폴로지 설정", description="맵 토폴로지 설정")
async def set_map_topology(robot_model: str, request: RequestSetMapTopologyPD) -> ResponseSetMapTopologyPD:
    return await amr_map_service.set_map_topology(robot_model, request)

@amr_map_router.get("/{robot_model}/map/getFile", summary="맵 파일 반환", description="맵 파일 반환")
async def get_map_file(robot_model: str, mapName: str, fileName: str) -> StreamingResponse:
    return await amr_map_service.get_map_file(robot_model, mapName, fileName)

@amr_map_router.get("/{robot_model}/map/getZip", summary="맵 압축 후 반환", description="맵 압축 후 반환")
async def get_map_zip(robot_model: str, mapName: str, zipName: str) -> StreamingResponse:
    return await amr_map_service.get_map_zip(robot_model, mapName, zipName)

@amr_map_router.get("/map/getFileProgress/{transfer_id}", summary="맵 파일 진행 상태 조회", description="맵 파일 진행 상태 조회")
async def get_map_file_progress(transfer_id: str):
    return await amr_map_service.get_map_file_progress(transfer_id)

@amr_map_router.delete("/{robot_model}/map/delete", summary="맵 삭제", description="맵 삭제")
async def delete_map(robot_model: str, mapName: str):
    return await amr_map_service.delete_map(robot_model, mapName)

# @amr_map_router.get("/{robot_model}/map/loadMap", summary="맵 로드", description="맵 로드")
# async def load_map(robot_model: str, request: RequestMapLoadPD) -> ResponseMapLoadPD:
#     return await amr_map_service.load_map(robot_model, request)

# @amr_map_router.get("/{robot_model}/map/loadTopo", summary="토폴로지 맵 로드", description="토폴로지 맵 로드")
# async def load_topo_map(robot_model: str) -> ResponseTopoLoadPD:
#     return await amr_map_service.load_topo_map(robot_model)

# @amr_map_router.post("/{robot_model}/mapping/start", summary="매핑 시작", description="매핑 시작")
# async def mapping_start(robot_model: str) -> ResponseMappingStartPD:
#     return await amr_map_service.mapping_start(robot_model)

# @amr_map_router.post("/{robot_model}/mapping/stop", summary="매핑 중지", description="매핑 중지")
# async def mapping_stop(robot_model: str ) -> ResponseMappingStopPD:
#     return await amr_map_service.mapping_stop(robot_model)

# @amr_map_router.post("/{robot_model}/mapping/save", summary="매핑 저장", description="매핑 저장")
# async def mapping_save(robot_model: str, request: RequestMappingSavePD) -> ResponseMappingSavePD:
#     return await amr_map_service.mapping_save(robot_model, request)








# # @amr_map_router.get("/{robot_model}/map/file", summary="맵 파일 조회", description="맵 파일 조회")
# # async def get_map_file(robot_model: str, map_name: str, file_name: str) -> ResponseMapFilePD:
# #     return await amr_map_service.get_map_file(robot_model, map_name, file_name)
