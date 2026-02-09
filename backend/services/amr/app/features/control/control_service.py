import asyncio
from collections import defaultdict
from datetime import (
    UTC,
    datetime,
)

from rb_flat_buffers.SLAMNAV.ResultControlDock import ResultControlDockT
from rb_modules.log import rb_log
from rb_sdk.amr import RBAmrSDK
from rb_utils.parser import t_to_dict
from rb_utils.service_exception import ServiceException

from app.schema.amr import AmrResponseStatusEnum
from app.socket.socket_client import socket_client

from .adapter.mongo import ControlMongoDatabaseAdapter
from .adapter.smtplib import ControlSmtpLibEmailAdapter
from .control_schema import (
    RequestControlChargeTriggerPD,
    RequestControlDetectMarkerPD,
    RequestControlDockPD,
    RequestControlLedModePD,
    RequestControlSetObsBoxPD,
    RequestSetSafetyFieldPD,
    RequestSetSafetyFlagPD,
    RequestSetSafetyIoPD,
    ResponseControlChargeTriggerPD,
    ResponseControlDetectMarkerPD,
    ResponseControlDockPD,
    ResponseControlGetObsBoxPD,
    ResponseControlLedModePD,
    ResponseControlSetObsBoxPD,
    ResponseGetSafetyFieldPD,
    ResponseGetSafetyFlagPD,
    ResponseGetSafetyIoPD,
    ResponseSetSafetyFieldPD,
    ResponseSetSafetyFlagPD,
    ResponseSetSafetyIoPD,
)
from .domain.control import ControlModel

rb_amr_sdk = RBAmrSDK()
class AmrControlService:
    def __init__(self):
        # self.robot_model = "test"
        self.database_port = ControlMongoDatabaseAdapter()
        self.email_port = ControlSmtpLibEmailAdapter()
        self._locks = defaultdict(asyncio.Lock)


    async def control_dock(self, robot_model: str, request:RequestControlDockPD) -> ResponseControlDockPD:
        """
        [도킹 명령 전송]
        * robot_model : 명령을 전송할 로봇 모델
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_dock : {robot_model}, {request.command}")
            # 1) controlModel 객체 생성
            model.set_robot_model(robot_model)
            model.control_dock(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_dock] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.control.control_dock(
                robot_model=model.robot_model,
                command=model.command,
                req_id=model.id
            )

            print("============== control_dock result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_dock] DB Exception : ", e)

            print("============== control_dock return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_dock] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_dock_result(self, topic: str, result: ResultControlDockT) -> ResponseControlDockPD:
        """
        [도킹 명령 결과 처리]
        * topic : 도킹 명령 결과 토픽
        * obj : 도킹 명령 결과 객체
        """
        try:
            rb_log.info(f"[amr_control_service] control_dock_result : {topic}, {result}")

            # 1) database 조회
            db = await self.database_port.get_log_by_id(result.id)
            if db is not None:
                db["result"] = result.result
                db["message"] = result.message
            else:
                db = result.copy()
                db["createdAt"] = datetime.now(UTC)
                db["updateAt"] = datetime.now(UTC)

            if result.result == "fail" or result.result == "success":
                db["status"] = AmrResponseStatusEnum.DONE
            elif result.result == "cancel":
                db["status"] = AmrResponseStatusEnum.CANCEL
            elif result.result == "start":
                db["status"] = AmrResponseStatusEnum.MOVING
            elif result.result == "pause":
                db["status"] = AmrResponseStatusEnum.PAUSE

            # 2) database 저장
            try:
                await self.database_port.upsert(db)
            except ServiceException as e:
                print("[control_dock_result] DB Exception : ", e)

            # 3) 소켓 전송
            await socket_client.emit(topic, t_to_dict(result))
        except ServiceException as e:
            print("[control_dock_result] ServiceException : ", e.message, e.status_code)
            return None
        except Exception as e:
            print("[control_dock_result] Exception : ", e)
            return None


    async def control_chargeTrigger(self, robot_model: str, request:RequestControlChargeTriggerPD) -> ResponseControlChargeTriggerPD:
        """
        [충전 트리거 명령 전송]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 충전 트리거 명령 요청
        """

        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_chargeTrigger : {robot_model}, {request.control}")
            # 1) controlModel 객체 생성
            model.set_robot_model(robot_model)
            model.set_control_charge_trigger(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_dock] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.control.control_charge_trigger(
                robot_model=model.robot_model,
                req_id=model.id,
                control=model.control
            )

            print("============== control_charge_trigger result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_charge_trigger] DB Exception : ", e)

            print("============== control_charge_trigger return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_charge_trigger] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_get_safetyField(self, robot_model: str) -> ResponseGetSafetyFieldPD:
        """
        [안전 필드 조회]
        * robot_model : 명령을 전송할 로봇 모델
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_get_safetyField : {robot_model}")
            model.set_robot_model(robot_model)
            model.get_control_safety_field()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_get_safetyField] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_get_safety_field(
                robot_model=model.robot_model,
                req_id=model.id
            )

            print("============== control_get_safetyField result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_get_safetyField] DB Exception : ", e)

            print("============== control_get_safetyField return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_get_safetyField] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_set_safetyField(self, robot_model: str, request:RequestSetSafetyFieldPD) -> ResponseSetSafetyFieldPD:
        """
        [안전 필드 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 안전 필드 설정 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_set_safetyField : {robot_model}, {request.safety_field}")
            model.set_robot_model(robot_model)
            model.set_control_safety_field(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_set_safetyField] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_set_safety_field(
                robot_model=model.robot_model,
                req_id=model.id,
                safety_field=model.safety_field
            )

            print("============== control_set_safetyField result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_set_safetyField] DB Exception : ", e)

            print("============== control_set_safetyField return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_set_safetyField] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)

    async def control_get_safetyFlag(self, robot_model: str) -> ResponseGetSafetyFlagPD:
        """
        [안전 플래그 조회]
        * robot_model : 명령을 전송할 로봇 모델
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_get_safetyFlag : {robot_model}")
            model.set_robot_model(robot_model)
            model.get_control_safety_flag()
            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_get_safetyFlag] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_get_safety_flag(
                robot_model=model.robot_model,
                req_id=model.id
            )

            print("============== control_get_safetyFlag result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_get_safetyFlag] DB Exception : ", e)

            print("============== control_get_safetyFlag return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_get_safetyFlag] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_set_safetyFlag(self, robot_model: str, request:RequestSetSafetyFlagPD) -> ResponseSetSafetyFlagPD:
        """
        [안전 플래그 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 안전 플래그 설정 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_set_safetyFlag : {robot_model}, {request.safety_flag}")
            model.set_robot_model(robot_model)
            model.set_control_safety_flag(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_set_safetyFlag] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_set_safety_flag(
                robot_model=model.robot_model,
                req_id=model.id,
                reset_flag=model.safety_flag
            )

            print("============== control_set_safetyFlag result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_set_safetyFlag] DB Exception : ", e)

            print("============== control_set_safetyFlag return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_set_safetyFlag] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)

    async def control_led(self, robot_model: str, request:RequestControlLedModePD) -> ResponseControlLedModePD:
        """
        [LED 명령 전송]
        * robot_model : 명령을 전송할 로봇 모델
        * request : LED 명령 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_led : {robot_model}, {request.control}")
            model.set_robot_model(robot_model)
            model.set_control_led(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_led] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_led_mode(
                robot_model=model.robot_model,
                req_id=model.id,
                control=model.control,
                color=model.color
            )

            print("============== control_led result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_led] DB Exception : ", e)

            print("============== control_led return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_led] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_get_safetyIo(self, robot_model: str) -> ResponseGetSafetyIoPD:
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_get_safetyIo : {robot_model}")
            model.set_robot_model(robot_model)
            model.get_control_safety_io()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_get_safetyIo] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_get_safety_io(
                robot_model=model.robot_model,
                req_id=model.id
            )

            print("============== control_get_safetyIo result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_get_safetyIo] DB Exception : ", e)

            print("============== control_get_safetyIo return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_get_safetyIo] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_set_safetyIo(self, robot_model: str, request:RequestSetSafetyIoPD) -> ResponseSetSafetyIoPD:
        """
        [세이프티 IO 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 세이프티 IO 설정 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_set_safetyIo : {robot_model}, {request.mcu0Din}, {request.mcu1Din}")
            model.set_robot_model(robot_model)
            model.set_control_safety_io(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_set_safetyIo] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_set_safety_io(
                robot_model=model.robot_model,
                req_id=model.id,
                mcu0_din=model.mcu0_din,
                mcu1_din=model.mcu1_din
            )

            print("============== control_set_safetyIo result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_set_safetyIo] DB Exception : ", e)

            print("============== control_set_safetyIo return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_set_safetyIo] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_get_obsbox(self, robot_model: str) -> ResponseControlGetObsBoxPD:
        """
        [장애물 박스 조회]
        * robot_model : 명령을 전송할 로봇 모델
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_get_obsbox : {robot_model}")
            model.set_robot_model(robot_model)
            model.get_control_obs_box()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_get_obsbox] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_get_obs_box(
                robot_model=model.robot_model,
                req_id=model.id
            )

            print("============== control_get_obsbox result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_get_obsbox] DB Exception : ", e)

            print("============== control_get_obsbox return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_get_obsbox] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_set_obsbox(self, robot_model: str, request:RequestControlSetObsBoxPD) -> ResponseControlSetObsBoxPD:
        """
        [장애물 박스 설정]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 장애물 박스 설정 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_set_obsbox : {robot_model}, {request.minX}, {request.minY}, {request.minZ}, {request.maxX}, {request.maxY}, {request.maxZ}, {request.mapRange}")
            model.set_robot_model(robot_model)
            model.set_control_obs_box(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_set_obsbox] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_set_obs_box(
                robot_model=model.robot_model,
                req_id=model.id,
                min_x=model.min_x,
                min_y=model.min_y,
                min_z=model.min_z,
                max_x=model.max_x,
                max_y=model.max_y,
                max_z=model.max_z,
                map_range=model.map_range
            )

            print("============== control_set_obsbox result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_set_obsbox] DB Exception : ", e)

            print("============== control_set_obsbox return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_set_obsbox] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()

    async def control_detect(self, robot_model: str, request:RequestControlDetectMarkerPD) -> ResponseControlDetectMarkerPD:
        """
        [마커 감지 명령 전송]
        * robot_model : 명령을 전송할 로봇 모델
        * request : 마커 감지 명령 요청
        """
        model = ControlModel()
        try:
            rb_log.info(f"[amr_control_service] control_detect : {robot_model}, {request.control}")
            model.set_robot_model(robot_model)
            model.set_control_detect(request)

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_detect] DB Exception : ", e)

            # 3) 요청 전송
            result = await rb_amr_sdk.control.control_detect_marker(
                robot_model=model.robot_model,
                req_id=model.id,
                command=model.command,
                camera_number=model.camera_number,
                camera_serial=model.camera_serial,
                marker_size=model.marker_size
            )

            print("============== control_detect result ===============")
            print(result)

            model.result_change(result.get("result"))
            model.message = result.get("message")
            model.status_change(result.get("result"))

            try:
                await self.database_port.upsert(model.to_dict())
            except Exception as e:  # pylint: disable=broad-exception-caught
                print("[control_detect] DB Exception : ", e)

            print("============== control_detect return ===============")
            return model.to_dict()
        except ServiceException as e:
            print("[control_detect] ServiceException : ", e.message, e.status_code)
            model.status_change(AmrResponseStatusEnum.FAIL)
            model.message = str(e.message)
            return model.to_dict()
