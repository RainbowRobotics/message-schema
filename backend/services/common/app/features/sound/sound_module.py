import json

from fastapi.responses import JSONResponse
from rb_flat_buffers.IPC.Request_Sound_Play import Request_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Play import Response_Sound_PlayT
from rb_flat_buffers.IPC.Response_Sound_Stop import Response_Sound_StopT
from rb_flat_buffers.IPC.Response_Sound_Pause_Toggle import Response_Sound_Pause_ToggleT
from rb_modules.log import rb_log
from rb_modules.service import BaseService
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient

from rb_flat_buffers.IPC.Response_Sound_GetStatus import Response_Sound_GetStatusT

from app.features.sound.sound_schema import Request_Sound_PlayPD, Response_Sound_PlayPD, Response_Sound_StopPD

zenoh_client = ZenohClient()

class SoundService(BaseService):
    """
    Sound Service
    """

    async def get_status(self):
        """
        [현재 사운드 재생 상태 조회]
        """

        # 1) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "sound/status",
            flatbuffer_res_T_class=Response_Sound_GetStatusT,
            flatbuffer_buf_size=125,
            timeout=3
        )

        # 2) 에러 확인 및 반환
        if result.get("err") is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[sound_module] getStatus Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"]}
            )

        # 3) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})

    async def play_sound(self, request: Request_Sound_PlayPD):
        """
        [사운드 재생]
        """

        # 1) request 객체 생성
        req = Request_Sound_PlayT()
        req.fileName = request.fileName
        req.filePath = request.filePath
        req.volume = request.volume
        req.repeatCount = request.repeatCount

        # 2) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "sound/play",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Sound_PlayT,
            flatbuffer_buf_size=125,
            timeout=999999
        )
        print(">>>result: ", result, flush=True)

        # 3) 에러 확인 및 반환
        if result.get("err") is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[sound_module] getStatus Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"], "request": request.model_dump()}
            )

        # 3) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})


    async def stop_sound(self):
        """
        [사운드 종료]
        """
        # 1) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "sound/stop",
            flatbuffer_res_T_class=Response_Sound_StopT,
            flatbuffer_buf_size=125,
            timeout=3
        )

        # 2) 에러 확인 및 반환
        if result.get("err") is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[sound_module] getStatus Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"]}
            )

        # 3) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})

    async def pause_toggle(self):
        """
        [사운드 일시정지]
        """
        # 1) network 서비스로 요청 (zenoh)
        result = zenoh_client.query_one(
            "sound/pause",
            flatbuffer_res_T_class=Response_Sound_Pause_ToggleT,
            flatbuffer_buf_size=125,
            timeout=3
        )

        # 2) 에러 확인 및 반환
        if result.get("err") is not None:
            if isinstance(result["err"], str) and result["err"].lstrip().startswith(("{", "[")):
                error_result = json.loads(result["err"])
            else:
                error_result = {"code": 500, "message": result["err"]}
            rb_log.error(f"[sound_module] getStatus Error : {error_result}")
            return JSONResponse(
                status_code=error_result["code"],
                content={"message": error_result["message"]}
            )

        # 3) obj 반환 및 예외처리
        if result.get("obj_payload") is not None:
            return result["obj_payload"]
        else:
            return JSONResponse(status_code=500, content={"dict_payload":result.get("dict_payload"), "message":"요청 응답이 처리되지 않았습니다(obj_payload is None)"})
