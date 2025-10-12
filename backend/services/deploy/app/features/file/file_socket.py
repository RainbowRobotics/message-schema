from rb_socketio import RbSocketIORouter
from utils.parser import t_to_dict, to_json

from .file_module import FileService

file_socket_router = RbSocketIORouter()
file_service = FileService()


@file_socket_router.on("upload")
async def on_upload_file(data):
    dict_data = t_to_dict(data)
    res = await file_service.upload_file(dict_data)
    return to_json(res)


@file_socket_router.on("download")
async def on_download_file(data):
    dict_data = t_to_dict(data)
    res = await file_service.download_file(dict_data)
    return to_json(res)
