import contextlib
import hashlib
import os
import platform
import tempfile

from app.socket.socket_client import socket_client
from fastapi import HTTPException, UploadFile
from fastapi.responses import FileResponse
from rb_flat_buffers.deploy.Response_Deploy_Upload import Response_Deploy_UploadT
from rb_modules.log import rb_log
from rb_zenoh.client import ZenohClient
from utils.asyncio_helper import fire_and_log

zenoh_client = ZenohClient()


class FileService:
    def __init__(self):
        system = platform.system().lower()
        if system == "windows":
            self.ROOT = r"C:\rainbow\deploy"
        elif system == "darwin":
            self.ROOT = "/Users/Shared/rainbow/deploy"
        else:  # Linux or others
            self.ROOT = "/opt/rainbow/deploy"

        self.DEPLOY_TOKEN = os.environ.get("DEPLOY_TOKEN", "")
        self.MAX_ZIP_COUNT = 3

    def sha256_of(self, fpath):
        h = hashlib.sha256()
        with open(fpath, "rb") as f:
            for b in iter(lambda: f.read(1 << 20), b""):
                h.update(b)
        return h.hexdigest()

    def _get_inbox_path(self, sw_name: str, mode: str, service_name: str | None = None):
        if service_name:
            inbox = os.path.join(self.ROOT, "inbox", sw_name, service_name, mode)
        else:
            inbox = os.path.join(self.ROOT, "inbox", sw_name, mode)

        os.makedirs(inbox, exist_ok=True)
        return inbox

    async def upload_file(
        self,
        *,
        file: UploadFile,
        tag: str,
        mode: str,
        sw_name: str,
        extension: str = "zip",
        service_name: str | None = None,
    ):
        file_name = f"{tag}.{extension}"

        inbox = self._get_inbox_path(sw_name, mode, service_name)

        tmp_fd, tmp_path = tempfile.mkstemp(dir=inbox)

        with os.fdopen(tmp_fd, "wb") as out:
            while True:
                chunk = await file.read(4 * 1024 * 1024)
                if not chunk:
                    break
                out.write(chunk)

        final = os.path.join(inbox, file_name)
        os.rename(tmp_path, final)

        try:
            zips = [
                (name, os.path.getmtime(os.path.join(inbox, name)))
                for name in os.listdir(inbox)
                if name.endswith(f".{extension}")
            ]

            zips.sort(key=lambda x: x[1], reverse=True)

            for name, _ in zips[self.MAX_ZIP_COUNT :]:
                path = os.path.join(inbox, name)

                with contextlib.suppress(FileNotFoundError):
                    os.remove(path)

            rb_log.info(f"[Deploy Upload] Removed '{sw_name}/{mode}' old build {extension} file")
        except Exception:
            rb_log.error(
                f"[Deploy Upload] Failed to remove '{sw_name}/{mode}' old build {extension} file"
            )
            pass

        res = {
            "tag": tag,
            "file_name": file_name,
            "sha256": self.sha256_of(final),
            "path": final,
        }

        req = Response_Deploy_UploadT()
        req.tag = res["tag"]
        req.fileName = res["file_name"]
        req.sha256 = res["sha256"]
        req.path = res["path"]

        topic_name = f"{sw_name}/upload"

        fire_and_log(socket_client.emit(topic_name, res))
        zenoh_client.publish(topic_name, flatbuffer_req_obj=req, flatbuffer_buf_size=320)

        return res

    async def download_file(
        self,
        *,
        tag: str,
        mode: str,
        sw_name: str,
        extension: str = "zip",
        service_name: str | None = None,
    ):
        file_name = f"{tag}.{extension}"

        inbox = self._get_inbox_path(sw_name, mode, service_name)

        path = os.path.join(inbox, file_name)

        if not os.path.exists(path):
            raise HTTPException(404, "not found")

        return FileResponse(path, filename=file_name)
