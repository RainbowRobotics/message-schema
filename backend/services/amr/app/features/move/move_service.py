"""
[AMR 이동 서비스]
"""
import asyncio
import json
import os
from collections import (
    defaultdict,
)
from datetime import (
    UTC,
    datetime,
)
from pathlib import Path
from typing import Any

# pylint: disable=import-error,no-name-in-module
from fastapi import BackgroundTasks
from fastapi.encoders import jsonable_encoder
from fastapi.responses import FileResponse, JSONResponse
from rb_modules.log import rb_log  # pylint: disable=import-error,no-name-in-module
from rb_sdk.amr import RBAmrSDK
from rb_utils.date import convert_dt  # pylint: disable=import-error,no-name-in-module
from rb_utils.parser import t_to_dict
from rb_utils.service_exception import (
    ServiceException,  # pylint: disable=import-error,no-name-in-module
)

from app.features.move.adapter.mongo import (
    MoveMongoDatabaseAdapter,
)
from app.features.move.adapter.smtplib import (
    MoveSmtpLibEmailAdapter,
)
from app.features.move.domain.move import MoveModel
from app.features.move.move_schema import (
    Request_Move_ArchiveLogPD,
    Request_Move_CircularPD,
    Request_Move_ExportLogPD,
    Request_Move_GoalPD,
    Request_Move_LinearPD,
    Request_Move_LogsPD,
    Request_Move_RotatePD,
    Request_Move_TargetPD,
    RequestMoveJogPD,
    Response_Move_CircularPD,
    Response_Move_GoalPD,
    Response_Move_LinearPD,
    Response_Move_LogsPD,
    Response_Move_PausePD,
    Response_Move_ResumePD,
    Response_Move_RotatePD,
    Response_Move_StopPD,
    Response_Move_TargetPD,
)
from app.schema.amr import AmrResponseStatusEnum
from app.socket.socket_client import socket_client

rb_amr_sdk = RBAmrSDK()
class AmrMoveService:
    """
    [AMR 이동 서비스 초기화]
    """
    def __init__(self):
        self.database_port = MoveMongoDatabaseAdapter()
        self.email_port = MoveSmtpLibEmailAdapter()
        self._locks = defaultdict(asyncio.Lock)

    async def move_goal(self, robot_model: str, robot_id: str, req: Request_Move_GoalPD, model: MoveModel | None = None) -> Response_Move_GoalPD:
        """
        [AMR 목표 지점으로 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveGoal : {robot_model} {robot_id} {req.model_dump()}")

            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_goal(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[moveGoal] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_goal(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                goal_id=model.goal_id,
                goal_name=model.goal_name,
                method=model.method,
                preset=model.preset
            )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveGoal] DB Exception : ", e)

            return model.to_dict()

        except ServiceException as e:
            print("[moveGoal] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))


    async def move_target(self,
    robot_model: str,
    robot_id: str,
    req: Request_Move_TargetPD,
    model:MoveModel | None = None) -> Response_Move_TargetPD:
        """
        [AMR 타겟 좌표로 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveTarget : {robot_model} {robot_id} {req.model_dump()}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_target(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveGoal] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_target(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                x=model.goal_pose[0],
                y=model.goal_pose[1],
                z=model.goal_pose[2],
                rz=model.goal_pose[3],
                method=model.method,
                preset=model.preset
            )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveTarget] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveTarget] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_jog(self, robot_model: str, robot_id: str, request: RequestMoveJogPD):
        """
        [AMR 조이스틱 이동]
        """
        model = MoveModel()
        try:
            # rb_log.info(f"[amr_move_service] moveJog : {robot_model} {robot_id} {request.model_dump()}")

            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_jog(request)

            # 2) DB 저장 (패스)

            # 3) 요청 검사
            # model.check_variables()

            # 4) 요청 전송
            await rb_amr_sdk.move.send_move_jog(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                vx=model.vx,
                vy=model.vy,
                wz=model.wz
            )

            return None

        except ServiceException as e:
            print("[moveJog] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_stop(self, robot_model: str, robot_id: str) -> Response_Move_StopPD:
        """
        [AMR 이동 중지]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveStop : {robot_model} {robot_id}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_stop()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[moveGoal] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_stop(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id
            )

            print(f"=> move_stop result: {result}")
            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveStop] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveStop] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_pause(self, robot_model: str, robot_id: str, model: MoveModel | None = None) -> Response_Move_PausePD:
        """
        [AMR 이동 일시정지]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] movePause : {robot_model} {robot_id}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_pause()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[movePause] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_pause(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[movePause] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[movePause] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()} ))

    async def move_resume(self, robot_model: str, robot_id: str, model: MoveModel | None = None) -> Response_Move_ResumePD:
        """
        [AMR 이동 재개]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveResume : {robot_model} {robot_id}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_resume()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:
                print("[moveResume] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_resume(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveResume] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveResume] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_x_linear(self, robot_model: str, robot_id: str, req: Request_Move_LinearPD, model: MoveModel | None = None) -> Response_Move_LinearPD:
        """
        [AMR 선형 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] move xLinear : {robot_model} {robot_id} {req.model_dump()}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_x_linear(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[move xLinear] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_x_linear(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                target=model.target,
                speed=model.speed
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[move xLinear] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[move xLinear] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_y_linear(self, robot_model: str, robot_id: str, req: Request_Move_LinearPD, model: MoveModel | None = None) -> Response_Move_LinearPD:
        """
        [AMR 선형 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] move yLinear : {robot_model} {robot_id} {req.model_dump()}")

            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_y_linear(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[move yLinear] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_y_linear(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                target=model.target,
                speed=model.speed
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[move yLinear] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveLinear] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_circular(self, robot_model: str, robot_id: str, req: Request_Move_CircularPD, model: MoveModel | None = None) -> Response_Move_CircularPD:
        """
        [AMR 원형 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveCircular : {robot_model} {robot_id} {req.model_dump()}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_circular(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model)
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveCircular] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_circular(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                target=model.target,
                speed=model.speed,
                direction=model.direction
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveCircular] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveCircular] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def move_rotate(self, robot_model: str, robot_id: str, req: Request_Move_RotatePD, model: MoveModel | None = None) -> Response_Move_RotatePD:
        """
        [AMR 원형 이동]
        """
        model = MoveModel()
        try:
            rb_log.info(f"[amr_move_service] moveRotate : {robot_model} {robot_id} {req.model_dump()}")
            # 1) moveModel 객체 생성
            model.set_robot_model(robot_model, robot_id)
            model.set_move_rotate(req)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model)
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveRotate] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.move.send_move_rotate(
                robot_model=model.robot_model,
                robot_id=model.robot_id,
                req_id=model.id,
                target=model.target,
                speed=model.speed
                )

            model.result_change(result.result)
            model.message = result.message
            model.status_change(result.result)

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[moveRotate] DB Exception : ", e)

            return model.to_dict()
        except ServiceException as e:
            print("[moveRotate] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            await self.database_port.upsert(model.to_dict())
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "model": model.to_dict()}))

    async def get_logs(
        self,
        request: Request_Move_LogsPD
        ) -> Response_Move_LogsPD:
        """
        [AMR 이동 로그 조회]
        """
        try:
            rb_log.info(f"[amr_move_service] getLogs : {request.model_dump()}")

            # 1) 요청 검사
            if request.sort is None or request.sort == "":
                request.sort = "createdAt"
            if request.order is None or request.order == "":
                request.order = "desc"
            if request.page is None or request.page == 0:
                request.page = 1
            if request.limit is None or request.limit == 0:
                request.limit = 10

            result = await self.database_port.get_logs(request.model_dump())
            return result
        except ServiceException as e:
            rb_log.error(f"[getLogs] ServiceException : {e.message}, {e.status_code}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "request": request.model_dump()}))

    async def archive_logs(self, request: Request_Move_ArchiveLogPD):
        """
        [AMR 이동 로그 아카이브]
        """

        #1) 요청 검사
        try:
            if request.cutOffDate is None or request.cutOffDate == "":
                raise ServiceException("삭제 기준 날짜가 없습니다", status_code=400)
            if convert_dt(request.cutOffDate, "Asia/Seoul", "UTC") > datetime.now(UTC):
                raise ServiceException("삭제 기준 날짜가 오늘 이후입니다", status_code=400)
            if request.isDryRun is None or request.isDryRun == "":
                request.isDryRun = False

            rb_log.info(f"[amr_move_service] archiveLogs : {request}")

            result = await self.database_port.archive_logs(cutoff_utc=request.cutOffDate, dry_run=request.isDryRun)

            print(f"[archiveLogs] result: {result}")
            return result
        except ServiceException as e:
            rb_log.error(f"[archiveLogs] ServiceException : {e.message}, {e.status_code}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "request": request.model_dump()}))

    async def export_logs(self, request: Request_Move_ExportLogPD, background_tasks: BackgroundTasks):
        """
        [AMR 이동 로그 내보내기]
        """
        try:
            rb_log.info(f"[amr_move_service] exportLogs : {request.model_dump()}")
            # 1) 요청 검사
            if request.startDt is None:
                request.startDt = convert_dt(datetime(2025, 1, 1), "Asia/Seoul", "UTC")
            else:
                request.startDt = convert_dt(request.startDt, "Asia/Seoul", "UTC")

            if request.endDt is None:
                request.endDt = convert_dt(datetime.now(UTC), "Asia/Seoul", "UTC")
            else:
                request.endDt = convert_dt(request.endDt, "Asia/Seoul", "UTC")

            if request.startDt > request.endDt:
                raise ServiceException("시작 날짜가 종료 날짜보다 이후입니다", status_code=400)

            if request.startDt < convert_dt(datetime(2025, 1, 1), "UTC", "UTC"):
                raise ServiceException("시작 날짜가 2025년 1월 1일 이전입니다", status_code=400)

            # if request.endDt > datetime.now(UTC):
            #     request.endDt = datetime.now(UTC)

            # 2) 데이터 조회 및 파일 생성
            result = await self.database_port.export_logs(
                start_dt=request.startDt,
                end_dt=request.endDt,
                filters=request.filters,
                filename=request.filename,
                search_text=request.searchText,
                fields=request.fields,
                sort=request.sort,
                order=request.order
                )

            rb_log.debug(f"[exportLogs] result: {result}")

            if(result.get("estimatedDocs") == 0):
                raise ServiceException("내보낼 데이터가 없습니다", status_code=400)

            file_path = result.get("file") if result.get("file") else None
            if file_path is not None and file_path != "":
                p = Path(file_path)
                if not p.exists():
                    raise ServiceException("내보내기 파일 생성 실패", status_code=500)
            else:
                raise ServiceException("내보낼 데이터가 없습니다", status_code=400)

            download_name = request.filename or p.name
            if not download_name.endswith(".ndjson.gz"):
                download_name += ".ndjson.gz"
            download_name = os.path.basename(download_name)  # 디렉터리 traversal 방지

            # 2) 파일 내보내는 방법 분기
            if request.method == "email":
                if not request.email:
                    raise ServiceException("email 필드가 필요합니다 (method=email)", status_code=400)
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
                raise ServiceException("내보내기 방식 오류: 지원하지 않는 방식",status_code=400)
        except ServiceException as e:
            rb_log.error(f"[exportLogs] ServiceException : {e.message}, {e.status_code}")
            return JSONResponse(status_code=e.status_code,content=jsonable_encoder({"message": e.message, "request": request.model_dump()}))


    #########################################################
    # 내부 함수
    #########################################################

    async def move_result(self, topic:str, obj:dict):
        """
        [AMR 이동 상태 변경 처리]
        """
        rb_log.info(f"[amr_move_service] move_result : {obj.get('id')}, {obj.get('command')}, {obj.get('result')}")
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
                db["status"] = AmrResponseStatusEnum.DONE
            elif obj["result"] == "cancel":
                db["status"] = AmrResponseStatusEnum.CANCEL
            elif obj["result"] == "start":
                db["status"] = AmrResponseStatusEnum.MOVING
            elif obj["result"] == "pause":
                db["status"] = AmrResponseStatusEnum.PAUSE

            await self.database_port.upsert(db)
            await socket_client.emit(topic, t_to_dict(obj))

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
