from fastapi import UploadFile
from pydantic import BaseModel


class Request_File_UploadPD(BaseModel):
    file: UploadFile
    tag: str
    mode: str
    sw_name: str
    extension: str | None = "zip"
    service_name: str | None = None


class Response_File_UploadPD(BaseModel):
    tag: str
    file_name: str
    sha256: str
    path: str


class Response_File_DownloadPD(BaseModel):
    tag: str
    mode: str
    sw_name: str
    extension: str | None = "zip"
    service_name: str | None = None
