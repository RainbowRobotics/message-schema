"""
[AMR 이동 서비스]
"""
import asyncio
import json
from collections import (
    defaultdict,
)
from contextlib import (
    suppress,
)
from datetime import (
    UTC,
    datetime,
)
from pathlib import Path
from functools import wraps
from typing import Any
import os
from fastapi import HTTPException, BackgroundTasks
from fastapi.responses import FileResponse, JSONResponse
import rb_database.mongo_db as mongo_db
from rb_modules.log import (
    rb_log,
)
from rb_utils.date import (
    convert_dt,
)
from rb_utils.service_exception import (
    ServiceException,
)
from app.features.move.src.adapters.output.smtplib import (
    MoveSmtpLibEmailAdapter,
)

from app.features.move.schema.move_api import (
    Request_Move_GoalPD,
    Request_Move_JogPD,
    Request_Move_TargetPD,
    RequestAmrMoveArchiveLogPD,
    RequestAmrMoveExportLogPD,
    RequestAmrMoveLogsPD,
    Response_Move_GoalPD,
    Response_Move_LogsPD,
    Response_Move_PausePD,
    Response_Move_ResumePD,
    Response_Move_StopPD,
    Response_Move_TargetPD,
)
from app.features.move.src.adapters.output.mongo import (
    MoveMongoDatabaseAdapter,
)
from app.features.move.src.adapters.output.zenoh import (
    SlamnavZenohAdapter,
)
from app.features.move.src.domain.move_model import (
    MoveModel,
    MoveStatus,
)
from app.socket.socket_client import (
    socket_client,
)


def handle_move_error(fn):
    @wraps(fn)
    async def wrapper(self,*args, **kwargs):
        model = MoveModel()
        try:
            result = await fn(self, *args, **kwargs, model=model)
            return result
        except ServiceException as e:
            # 에러 발생 시, 상태 변경 및 실패로 저장
            print("[moveGoal] ServiceException : ", e.message, e.status_code)
            model.status_change(MoveStatus.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(mongo_db.db, model.to_dict())
            return JSONResponse(status_code=e.status_code,content=model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveGoal] Exception : ", e)
            model.status_change(MoveStatus.FAIL)
            model.message = str(e)
            await self.database_port.upsert(mongo_db.db, model.to_dict())
            return JSONResponse(status_code=500,content=model.to_dict())
    return wrapper

class AmrMoveService:
    """
    [AMR 이동 서비스 초기화]
    """
    def __init__(self):
        self.database_port = MoveMongoDatabaseAdapter()
        self.slamnav_port = SlamnavZenohAdapter()
        self.email_port = MoveSmtpLibEmailAdapter()
        self._locks = defaultdict(asyncio.Lock)

    async def move_state_change(self, topic:str, obj:dict):
        """
        [AMR 이동 상태 변경 처리]
        """
        rb_log.info(f"[amr_move_service] move_state_change : {obj.get('id')}, {obj.get('command')}, {obj.get('result')}")
        _id = obj.get("id")
        if not _id:
            return

        async with self._locks[_id]:
            db = await self.database_port.get_log_by_id(obj["id"])
            if db is not None:
                db["result"] = obj["result"]
                db["message"] = obj["message"]
            else:
                db = obj.copy()
                db["createdAt"] = datetime.now(UTC)
                db["updateAt"] = datetime.now(UTC)

            if obj["result"] == "fail" or obj["result"] == "success":
                db["status"] = MoveStatus.DONE
            elif obj["result"] == "cancel":
                db["status"] = MoveStatus.CANCEL
            elif obj["result"] == "start":
                db["status"] = MoveStatus.MOVING
            elif obj["result"] == "pause":
                db["status"] = MoveStatus.PAUSE

            await self.database_port.upsert(db)
            await socket_client.emit(topic, obj)

    @handle_move_error
    async def move_goal(self, req: Request_Move_GoalPD, model: MoveModel | None = None) -> Response_Move_GoalPD:
        """
        [AMR 목표 지점으로 이동]
        """
        rb_log.info(f"[amr_move_service] moveGoal : {req.model_dump()}")
        # 1) moveModel 객체 생성
        model.set_move_goal(req)

        # 2) DB 저장
        try:
            await self.database_port.upsert(model.to_dict())
        except ServiceException as e:
            print("[moveGoal] DB Exception : ", e)

        # 3) 요청 검사
        model.check_variables()

        # 4) 요청 전송
        result = await self.slamnav_port.send_move_goal(model)

        model.result_change(result.result)
        model.message = result.message
        model.status_change(result.result)

        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveGoal] DB Exception : ", e)
        return model.to_dict()

    async def move_target(self,
    req: Request_Move_TargetPD,
    model:MoveModel | None = None) -> Response_Move_TargetPD:
        """
        [AMR 타겟 좌표로 이동]
        """
        rb_log.info(f"[amr_move_service] moveTarget : {req.model_dump()}")
        # 1) moveModel 객체 생성
        model.set_move_target(req)

        # 2) DB 저장
        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveGoal] DB Exception : ", e)

        # 3) 요청 검사
        model.check_variables()

        # 4) 요청 전송
        result = await self.slamnav_port.send_move_target(model)

        model.result_change(result.result)
        model.message = result.message
        model.status_change(result.result)

        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveTarget] DB Exception : ", e)

        return model.to_dict()

    async def move_jog(self, req: Request_Move_JogPD, model:MoveModel | None = None):
        """
        [AMR 조이스틱 이동]
        """
        rb_log.info(f"[amr_move_service] moveJog : {req.model_dump()}")
        try:
            # 1) moveModel 객체 생성
            model.set_move_jog(req)

            # 2) DB 저장 (패스)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            await self.slamnav_port.send_move_jog(model)
        except ServiceException as e:
            print("[moveJog] ServiceException : ", e.message, e.status_code)
            return JSONResponse(status_code=e.status_code,content=model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            return {
                "vx": req.vx if 'req' in locals() else 0,
                "vy": req.vy if 'req' in locals() else 0,
                "wz": req.wz if 'req' in locals() else 0,
                "result": "error",
                "message": f"Qt 프로그램으로 이동 명령 전송 실패: {str(e)}"
            }

    @handle_move_error
    async def move_stop(self, model: MoveModel | None = None) -> Response_Move_StopPD:
        """
        [AMR 이동 중지]
        """
        rb_log.info(f"[amr_move_service] moveStop : {model.model_dump()}")
        # 1) moveModel 객체 생성
        model.set_move_stop()

        # 2) DB 저장
        with suppress(Exception):  # pylint: disable=broad-exception-caught  # 필요하면 유지
            await self.database_port.upsert(model.to_dict())

        # 3) 요청 검사
        model.check_variables()

        # 4) 요청 전송
        result = await self.slamnav_port.send_move_stop(model)

        model.result_change(result.result)
        model.message = result.message
        model.status_change(result.result)
        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveStop] DB Exception : ", e)
        return model.to_dict()

    @handle_move_error
    async def move_pause(self, model: MoveModel | None = None) -> Response_Move_PausePD:
        """
        [AMR 이동 일시정지]
        """
        rb_log.info(f"[amr_move_service] movePause : {model.model_dump()}")
        # 1) moveModel 객체 생성
        model.set_move_pause()

        # 2) DB 저장
        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[movePause] DB Exception : ", e)

        # 3) 요청 검사
        model.check_variables()

        # 4) 요청 전송
        result = await self.slamnav_port.send_move_pause(model)

        model.result_change(result.result)
        model.message = result.message
        model.status_change(result.result)

        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[movePause] DB Exception : ", e)
        return model.to_dict()

    @handle_move_error
    async def move_resume(self, model: MoveModel | None = None) -> Response_Move_ResumePD:
        """
        [AMR 이동 재개]
        """
        rb_log.info(f"[amr_move_service] moveResume : {model.model_dump()}")
        # 1) moveModel 객체 생성
        model.set_move_resume()

        # 2) DB 저장
        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:
            print("[moveResume] DB Exception : ", e)
            raise e

        # 3) 요청 검사
        model.check_variables()

        # 4) 요청 전송
        result = await self.slamnav_port.send_move_resume(model)

        model.result_change(result.result)
        model.message = result.message
        model.status_change(result.result)

        try:
            await self.database_port.upsert(model.to_dict())
        except Exception as e:  # pylint: disable=broad-exception-caught
            print("[moveResume] DB Exception : ", e)
        return model.to_dict()


    def _parse_json_maybe(self, v: str | dict[str, Any] | None) -> dict[str, Any] | None:
        if v is None:
            return None
        if isinstance(v, dict):
            return v
        # 쿼리로 JSON 문자열이 들어온 경우
        try:
            return json.loads(v)
        except Exception:  # pylint: disable=broad-exception-caught
            # 잘못된 JSON은 무시하거나 400 에러를 던져도 됨
            return None

    def _normalize_projection(self, proj: dict[str, Any] | None) -> dict[str, Any] | None:
        if not proj:
            return None
        # _id만 제외하고 나머지 포함/제외 혼용 방지
        vals = [v for k, v in proj.items() if k != "_id"]
        if vals:
            is_include = all(v in (1, True) for v in vals)
            is_exclude = all(v in (0, False) for v in vals)
            if not (is_include or is_exclude):
                # 혼용되면 projection 무시(혹은 400 에러)
                return None
        return proj

    async def get_logs(
        self,
        request: RequestAmrMoveLogsPD
        ) -> Response_Move_LogsPD:
        """
        [AMR 이동 로그 조회]
        """
        rb_log.info(f"[amr_move_service] getLogs : {request.model_dump()}")
        try:
            # 1) 요청 검사
            result = await self.database_port.get_logs(request.model_dump())
            return result
        except ServiceException as e:
            raise e
        except Exception as e:  # pylint: disable=broad-exception-caught
            print(f"[getLogs] Exception : {e}")
            raise ServiceException("로그 조회 실패", status_code=500) from e



    async def archive_logs(self, request: RequestAmrMoveArchiveLogPD):
        """
        [AMR 이동 로그 아카이브]
        """
        rb_log.info(f"[amr_move_service] archiveLogs : {request.model_dump()}")
        result = await self.database_port.archive_logs(cutoff_utc=request.cut_off_date, dry_run=request.is_dry_run)
        print(f"[archiveLogs] result: {result}")
        return result



    async def export_logs(self, request: RequestAmrMoveExportLogPD, background_tasks: BackgroundTasks):
        """
        [AMR 이동 로그 내보내기]
        """
        rb_log.info(f"[amr_move_service] exportLogs : {request.model_dump()}")
        try:
            # 1) 요청 검사
            if request.start_dt is None:
                request.start_dt = convert_dt(datetime(2025, 1, 1), "Asia/Seoul", "UTC")
            if request.end_dt is None:
                request.end_dt = convert_dt(datetime.now(UTC), "Asia/Seoul", "UTC")
            request.start_dt = convert_dt(request.start_dt, "UTC", "UTC")
            request.end_dt = convert_dt(request.end_dt, "UTC", "UTC")

            if request.start_dt > request.end_dt:
                raise ServiceException("시작 날짜가 종료 날짜보다 큽니다", status_code=400)
            if request.start_dt < convert_dt(datetime(2025, 1, 1), "UTC", "UTC"):
                raise ServiceException("시작 날짜가 2025년 1월 1일 이전입니다", status_code=400)
            if request.end_dt > datetime.now(UTC):
                raise ServiceException("종료 날짜가 오늘 이후입니다", status_code=400)

            # 2) 파일 검사 및 임시파일 저장
            result = await self.database_port.export_logs(options=request.model_dump(exclude_none=True))
            print(f"[exportLogs] result: {result}")
            # return result

            file_path = result["file"]
            p = Path(file_path)
            if not p.exists():
                raise HTTPException(status_code=500, detail="내보내기 파일 생성 실패")

            download_name = request.filename or p.name
            if not download_name.endswith(".ndjson.gz"):
                download_name += ".ndjson.gz"
            download_name = os.path.basename(download_name)  # 디렉터리 traversal 방지

            # 2) 파일 내보내는 방법 분기
            if request.method == "email":
                if not request.email:
                    raise HTTPException(status_code=400, detail="email 필드가 필요합니다 (method=email)")
                # 백그라운드 메일 발송 트리거
                background_tasks.add_task(self.email_port.send_export_email, request.email, str(p), download_name)
                # await self.email_port.send_export_email(request.email, str(p), download_name)
                return {
                    "ok": True,
                    "message": "메일 발송 요청 완료",
                    "estimatedDocs": result.get("estimatedDocs"),
                    "archivedDocs": result.get("archivedDocs"),
                    # "filename": download_name,
                    "size": result.get("size"),
                }
            elif request.method == "file":
                # NDJSON + gzip
                headers = {
                    # 내용은 NDJSON, 전송은 gzip
                    "Content-Type": "application/gzip",
                }
                # FileResponse는 알아서 async로 처리됨
                return FileResponse(
                    path=str(p),
                    media_type="application/gzip",
                    filename=download_name,     # Content-Disposition 세팅
                    headers=headers,
                )
            else:
                raise HTTPException(status_code=400, detail="내보내기 방식 오류: 지원하지 않는 방식")
        except ServiceException as e:
            print(f"[exportLogs] ServiceException : {e.message}, {e.status_code}")
            raise e
        except Exception as e:
            print(f"[exportLogs] Exception : {e}")
            raise ServiceException("내보내기 실패", status_code=500) from e
