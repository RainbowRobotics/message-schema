import uuid
from rb_sdk.amr import RBAmrSDK
from rb_modules.log import rb_log
from .map_schema import RequestMapListPD, ResponseMapCloudPD
from rb_utils.parser import t_to_dict

rb_amr_sdk = RBAmrSDK()

class AmrMapService:
    def __init__(self):
        pass

    async def get_map_list(self, robot_model: str):
        return await rb_amr_sdk.map.get_map_list(robot_model, str(uuid.uuid4()))

    async def get_map_cloud(self, robot_model: str, map_name: str, file_name: str):
        resp = await rb_amr_sdk.map.get_map_cloud(robot_model, str(uuid.uuid4()), map_name, file_name)
        data = t_to_dict(resp)
        rb_log.info(f"=============> get_map_cloud resp: {data.get('rowCount')} {data.get('columnCount')}")

        new_data:list[list[float]] = []
        for i in range(data.get('rowCount')):
            new_data.append([data.get('data')[i * data.get('columnCount') + j] for j in range(data.get('columnCount'))])

        return ResponseMapCloudPD(
            mapName=data.get('mapName'),
            fileName=data.get('fileName'),
            data=new_data,
            columnCount=data.get('columnCount'),
            rowCount=data.get('rowCount'),
            result=data.get('result'),
            message=data.get('message', 'success')
        )
