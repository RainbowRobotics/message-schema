import uuid
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
from rb_sdk.amr import RBAmrSDK
from rb_modules.log import rb_log
from rb_utils.service_exception import ServiceException
from .map_schema import RequestSetMapCloudPD, ResponseGetMapCloudPD, ResponseGetMapTopologyPD, ResponseMapListPD, RequestGetMapCloudPD, ResponseSetMapCloudPD
from rb_utils.parser import t_to_dict
from .domain.map import MapModel

rb_amr_sdk = RBAmrSDK()

class AmrMapService:
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

    async def get_map_topology(self, robot_model: str, mapName: str, fileName: str) -> ResponseGetMapTopologyPD:
        """
        [AMR Map Topology 조회]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 요청 데이터
        """
        model = MapModel(robot_model)
        try:
            rb_log.info(f"[map_service] get_map_topology : {robot_model} {mapName} {fileName}")

            # 1) MapModel 설정
            model.get_map_topology(mapName, fileName)

            # 2) 요청 검사
            model.check_variables()

            # 3) 요청 전송
            result = await rb_amr_sdk.map.get_map_topology(robot_model, model.id, model.map_name, model.file_name)
            model.result = result.result
            model.message = result.message
            model.parse_topology_data(result.data)

            # 4) 반환
            return model.to_dict()
        except ServiceException as e:
            rb_log.error(f"[map_service] get_map_topology : {e}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))
