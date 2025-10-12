from app.features.file.file_module import FileService
from app.features.file.file_schema import (
    Response_File_DownloadPD,
    Response_File_UploadPD,
)
from fastapi import APIRouter, File, Form, UploadFile

file_service = FileService()
file_router = APIRouter(tags=["File"])


@file_router.post("/upload", response_model=Response_File_UploadPD)
async def upload_file(
    file: UploadFile = File(..., description="업로드할 파일"),  # noqa: B008
    tag: str = Form(...),
    mode: str = Form(...),
    sw_name: str = Form(...),
    extension: str = Form("zip"),
    service_name: str | None = Form(None),
):
    return await file_service.upload_file(
        file=file,
        tag=tag,
        mode=mode,
        sw_name=sw_name,
        extension=extension,
        service_name=service_name,
    )


@file_router.get("/download", response_model=Response_File_DownloadPD)
async def download_file(
    tag: str, mode: str, sw_name: str, extension: str = "zip", service_name: str | None = None
):
    return await file_service.download_file(
        tag=tag, mode=mode, sw_name=sw_name, extension=extension, service_name=service_name
    )
