import time

from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse, StreamingResponse
from rb_modules.log import rb_log
from rb_sdk.amr import RBAmrSDK
from rb_utils.parser import t_to_dict
from rb_utils.service_exception import ServiceException

from .domain.map import MapModel, TransferState
from .map_schema import (
    RequestSetMapCloudPD,
    RequestSetMapTopologyPD,
    ResponseGetMapCloudPD,
    ResponseGetMapTopologyPD,
    ResponseMapListPD,
    ResponseSetMapCloudPD,
    ResponseSetMapTopologyPD,
)

rb_amr_sdk = RBAmrSDK()

class AmrMapService:
    def __init__(self):
        self.transfer_store:dict[str, TransferState] = {}

    async def get_map_list(self, robot_model: str) -> ResponseMapListPD:
        """
        [AMR Map List 조회]
        * robot_model : 명령을 전송할 로봇 모델
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_list : {robot_model}")

            # 1) MapModel 설정
            model.get_map_list()

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.get_map_list(robot_model, req_id=model.id)
            model.map_list = result.list
            model.message = result.message
            model.result = result.result

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_list : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))


    async def get_map_cloud(self, robot_model: str, mapName: str, fileName: str) -> ResponseGetMapCloudPD:
        """
        [AMR Map Cloud 조회]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 요청 데이터
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_cloud : {robot_model} {mapName} {fileName}")

            # 1) MapModel 설정
            model.get_map_cloud(mapName, fileName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.get_map_cloud(robot_model, model.id, model.map_name, model.file_name)
            model.result = result.result
            model.message = result.message
            model.parse_cloud_data(result.data, result.columnCount, result.rowCount)

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_cloud : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def set_map_cloud(self, robot_model: str, request: RequestSetMapCloudPD) -> ResponseSetMapCloudPD:
        """
        [AMR Map Cloud 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 요청 데이터
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] set_map_cloud : {robot_model} {request.mapName} {request.fileName} {len(request.data)}")

            # 1) MapModel 설정
            model.set_map_cloud(request.mapName, request.fileName, request.data)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.set_map_cloud(
                robot_model,
                req_id =    model.id,
                map_name = model.map_name,
                file_name = model.file_name,
                data = model.cloud_data,
                column_count = model.cloud_column_count,
                row_count = model.cloud_row_count
            )
            model.result = result.result
            model.message = result.message

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] set_map_cloud : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def get_map_topology(self, robot_model: str, mapName: str | None = None, fileName: str | None = None, pageNo: int | None = None, pageSize: int | None = None, nodeType: str | None = None, searchText: str | None = None, sortOption: str | None = None, sortDirection: str | None = None) -> ResponseGetMapTopologyPD:
        """
        [AMR Map Topology 조회]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 요청 데이터
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_topology : {robot_model} {mapName} {fileName} {pageNo} {pageSize} {nodeType} {searchText} {sortOption} {sortDirection}")

            # 1) MapModel 설정
            model.get_map_topology(mapName, fileName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.get_map_topology(robot_model, model.id, model.map_name, model.file_name, pageNo, pageSize, nodeType, searchText, sortOption, sortDirection)

            model.result = result.result
            model.message = result.message
            model.parse_topology_data(result.data)
            model.page_no = result.pageNo
            model.page_size = result.pageSize
            model.total_page = result.totalPage
            model.node_type = result.nodeType
            model.search_text = result.searchText
            model.sort_option = result.sortOption
            model.sort_direction = result.sortDirection

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_topology : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def set_map_topology(self, robot_model: str, request: RequestSetMapTopologyPD) -> ResponseSetMapTopologyPD:
        """
        [AMR Map Topology 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 요청 데이터
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] set_map_topology : {robot_model} {request.mapName} {request.fileName} {len(request.data)}")

            # 1) MapModel 설정
            model.set_map_topology(request.mapName, request.fileName, request.data)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.set_map_topology(robot_model, model.id, model.map_name, model.file_name, model.get_node_t())

            model.result = result.result
            model.message = result.message

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] set_map_topology : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def get_map_file(self, robot_model: str, mapName: str, fileName: str) -> StreamingResponse:
        """
        [AMR Map File 조회]
        * robot_model : 명령을 전송할 로봇 모델
        * mapName : 맵 이름
        * fileName : 파일 이름
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_file : {robot_model} {mapName} {fileName}")

            # 1) MapModel 설정
            model.get_map_file(mapName, fileName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 메타 요청
            meta = await rb_amr_sdk.file.get_file_meta(robot_model, model.file_path, preferred_chunk_size=1024*1024*50)
            meta_dict = t_to_dict(meta)

            if meta_dict["result"].lower() != "accept":
                raise ServiceException(meta_dict["message"] or "file not found", status_code=404)

            # 4) 메타 데이터 설정
            total_chunks = meta_dict["totalChunks"]
            chunk_size = meta_dict["chunkSize"]
            file_size = meta_dict["fileSize"]
            mime = meta_dict["mime"] or "application/octet-stream"

            # 2) 진행률 상태 생성
            self.transfer_store[model.id] = TransferState(total=file_size, created_at=time.time())

            # 5) 스트리밍 제너레이터 선언
            async def gen():
                st = self.transfer_store[model.id]
                for idx in range(total_chunks):
                    chunk = await rb_amr_sdk.file.get_file_chunk(robot_model, model.file_path, idx, chunk_size)
                    chunk_dict = t_to_dict(chunk)
                    if chunk_dict["result"].lower() != "accept":
                        # 여기서 raise 하면 다운로드가 중간에 끊김(정상)
                        raise RuntimeError(chunk_dict["message"] or "chunk error")

                    data: bytes = bytes(chunk_dict["data"])

                    if len(data) > 0:
                        st.sent += len(data)
                        yield data
                    if bool(chunk_dict["eof"]):
                        break
                st.done = True

            # 6) 스트리밍 제너레이터 반환
            headers = {
                "X-Transfer-Id": model.id,
                "Content-Disposition": f'attachment; filename="{model.file_name}"',
                "Content-Length": str(file_size),
                "Content-Type": mime,
                "Cache-Control": "no-store",
            }
            return StreamingResponse(gen(), media_type="application/octet-stream", headers=headers)
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_file : {e}")
            st = self.transfer_store.get(model.id)
            if st:
                st.error = str(e)
                st.done = True
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message}))

    async def get_map_zip(self, robot_model: str, mapName: str, zipName: str) -> StreamingResponse:
        """
        [AMR Map Zip 조회]
        * robot_model : 명령을 전송할 로봇 모델
        * mapName : 맵 이름
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_zip : {robot_model} {mapName}")

            # 1) MapModel 설정
            model.get_map_zip(mapName, zipName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 압축 요청
            result = await rb_amr_sdk.file.compress_files(robot_model, base_dir=model.map_root_path, source_names=[model.map_name], output_name=model.file_name)
            result_dict = t_to_dict(result)
            if result_dict["result"].lower() != "accept":
                raise ServiceException(result_dict["message"] or "압축 실패", status_code=400)

            print("### 압축 result : ", result_dict)
            # 4) 메타 요청
            meta = await rb_amr_sdk.file.get_file_meta(robot_model, model.file_path, preferred_chunk_size=1024*1024*50)
            meta_dict = t_to_dict(meta)
            print("### 메타 result : ", meta_dict)

            if meta_dict["result"].lower() != "accept":
                raise ServiceException(meta_dict["message"] or "file not found", status_code=404)

            # 5) 메타 데이터 설정
            total_chunks = meta_dict["totalChunks"]
            chunk_size = meta_dict["chunkSize"]
            file_size = meta_dict["fileSize"]
            mime = meta_dict["mime"] or "application/octet-stream"

            # 6) 진행률 상태 생성
            self.transfer_store[model.id] = TransferState(total=file_size, created_at=time.time())

            # 7) 스트리밍 제너레이터 선언
            async def gen():
                st = self.transfer_store[model.id]
                for idx in range(total_chunks):
                    chunk = await rb_amr_sdk.file.get_file_chunk(robot_model, model.file_path, idx, chunk_size)
                    chunk_dict = t_to_dict(chunk)
                    if chunk_dict["result"].lower() != "accept":
                        # 여기서 raise 하면 다운로드가 중간에 끊김(정상)
                        raise RuntimeError(chunk_dict["message"] or "chunk error")

                    data: bytes = bytes(chunk_dict["data"])

                    if len(data) > 0:
                        st.sent += len(data)
                        yield data
                    if bool(chunk_dict["eof"]):
                        break
                st.done = True

            # 8) 스트리밍 제너레이터 반환
            headers = {
                "X-Transfer-Id": model.id,
                "Content-Disposition": f'attachment; filename="{model.file_name}"',
                "Content-Length": str(file_size),
                "Content-Type": mime,
                "Cache-Control": "no-store",
            }
            return StreamingResponse(gen(), media_type="application/octet-stream", headers=headers)
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_file : {e}")
            st = self.transfer_store.get(model.id)
            if st:
                st.error = str(e)
                st.done = True
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message}))

    async def delete_map(self, robot_model: str, mapName: str):
        """
        [AMR Map 삭제]
        * robot_model : 명령을 전송할 로봇 모델
        * mapName : 맵 이름
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] delete_map : {robot_model} {mapName}")

            # 1) MapModel 설정
            model.delete_map(mapName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.file.delete_file(robot_model, model.map_path)
            result_dict = t_to_dict(result)
            if result_dict["result"].lower() != "accept":
                raise ServiceException(result_dict["message"] or "삭제 실패", status_code=400)

            model.result = result_dict["result"]
            model.message = result_dict["message"]
            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] delete_map : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def get_map_file_progress(self, transfer_id: str):
        """
        [AMR Map File 진행 상태 조회]
        * transfer_id : 전송 ID
        """
        try:
            st = self.transfer_store.get(transfer_id)
            if not st:
                raise ServiceException("전송 정보를 찾을 수 없습니다", status_code=404)

            percent = 100.0 if st.total == 0 else (st.sent / st.total) * 100.0
            if percent > 100:
                percent = 100.0

            return {
                "transfer_id": transfer_id,
                "percent": round(percent, 2),
                "bytes_sent": st.sent,
                "total_bytes": st.total,
                "done": st.done,
                "error": st.error,
            }
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_file_progress : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message}))
