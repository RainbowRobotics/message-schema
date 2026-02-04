import asyncio
from collections import defaultdict

from rb_modules.log import rb_log
from rb_sdk.amr import RBAmrSDK
from rb_utils.service_exception import ServiceException

from app.schema.amr import AmrResponseStatusEnum

from .adapter.mongo import ControlMongoDatabaseAdapter
from .adapter.smtplib import ControlSmtpLibEmailAdapter
from .control_schema import (
    Request_Control_DetectPD,
    Request_Control_LEDPD,
    Request_Control_ObsBoxPD,
    Request_Control_DockPD,
    Request_Control_SafetyFieldPD,
    Request_Control_SafetyFlagPD,
    Request_Control_SafetyIOPD,
)
from .domain.control import ControlModel

rb_amr_sdk = RBAmrSDK()
class AmrControlService:
    def __init__(self):
        # self.robot_model = "test"
        self.database_port = ControlMongoDatabaseAdapter()
        self.email_port = ControlSmtpLibEmailAdapter()
        self._locks = defaultdict(asyncio.Lock)


    async def control_dock(self, robot_model: str, request:Request_Control_DockPD):
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

    async def control_chargeTrigger(self, robot_model: str):
        pass

    async def control_get_safetyField(self, robot_model: str):
        pass

    async def control_set_safetyField(self, robot_model: str, request:Request_Control_SafetyFieldPD):
        pass

    async def control_get_safetyFlag(self, robot_model: str):
        pass

    async def control_set_safetyFlag(self, robot_model: str, request:Request_Control_SafetyFlagPD):
        pass

    async def control_led(self, robot_model: str, request:Request_Control_LEDPD):
        pass

    async def control_get_safetyIo(self, robot_model: str):
        pass

    async def control_set_safetyIo(self, robot_model: str, request:Request_Control_SafetyIOPD):
        pass

    async def control_get_obsbox(self, robot_model: str):
        pass

    async def control_set_obsbox(self, robot_model: str, request:Request_Control_ObsBoxPD):
        pass

    async def control_detect(self, robot_model: str, request:Request_Control_DetectPD):
        pass
