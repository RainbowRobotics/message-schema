import os

from app.socket.socket_client import socket_client
from rb_flat_buffers.deploy.Response_Deploy_Progress import Response_Deploy_ProgressT
from rb_modules.log import rb_log
from rb_utils.asyncio_helper import fire_and_log
from rb_zenoh.client import ZenohClient

from .deploy_schema import DeployProgressSchemaPD

zenoh_client = ZenohClient()


class DeployService:
    def __init__(self):
        self.DEPLOY_TOKEN = os.environ.get("DEPLOY_TOKEN", "")

    def deploy_progress(
        self,
        request: DeployProgressSchemaPD,
    ):
        deploy_progress_data = {**request.model_dump()}

        sw_name = deploy_progress_data["sw_name"]
        ip = deploy_progress_data["ip"]
        mode = deploy_progress_data["mode"]
        tag = deploy_progress_data["tag"]
        percentage = deploy_progress_data["percentage"]
        service_name = deploy_progress_data["service_name"]
        result = deploy_progress_data["result"]

        if result not in ["success", "fail"]:
            error_msg = f"[Deploy Progress] sw_name: {sw_name}, result: {result} => result must be 'success' or 'fail'"
            rb_log.error(error_msg)
            raise ValueError(error_msg)

        rb_log.debug(f"[Deploy Progress] {sw_name}/{mode}/{service_name}/{tag} {percentage}%")

        payload = {
            "sw_name": sw_name,
            "mode": mode,
            "tag": tag,
            "percentage": percentage,
            "service_name": service_name,
            "result": result,
        }

        req = Response_Deploy_ProgressT()
        req.swName = payload["sw_name"]
        req.mode = payload["mode"]
        req.tag = payload["tag"]
        req.percentage = payload["percentage"]
        req.serviceName = payload["service_name"]
        req.result = payload["result"]

        topic_name = f"deploy/{sw_name}/{ip}/progress"

        fire_and_log(socket_client.emit(topic_name, req))
        zenoh_client.publish(topic_name, flatbuffer_req_obj=req, flatbuffer_buf_size=100)

        return req
