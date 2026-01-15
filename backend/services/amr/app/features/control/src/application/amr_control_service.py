import asyncio
from collections import defaultdict

from rb_modules.log import rb_log
from rb_sdk.amr import RBAmrSDK
from rb_utils.service_exception import ServiceException

from app.features.control.schema.control_api import (
    Request_Control_DetectPD,
    Request_Control_LEDPD,
    Request_Control_ObsBoxPD,
    Request_Control_SafetyFieldPD,
    Request_Control_SafetyFlagPD,
    Request_Control_SafetyIOPD,
)
from app.features.control.src.adapters.output.mongo import ControlMongoDatabaseAdapter
from app.features.control.src.adapters.output.smtplib import ControlSmtpLibEmailAdapter
from app.features.control.src.domain.control_model import ControlModel
from app.schema.amr import AmrResponseStatusEnum

rb_amr_sdk = RBAmrSDK()
class AmrControlService:
    def __init__(self):
        self.robot_model = "test"
        self.database_port = ControlMongoDatabaseAdapter()
        self.email_port = ControlSmtpLibEmailAdapter()
        self._locks = defaultdict(asyncio.Lock)


    async def control_dock(self):
        """
        """
        model = ControlModel()
        try:
            rb_log.info("[amr_control_service] control_dock")
            # 1) controlModel 객체 생성
            model.control_dock()

            # 2) DB 저장
            try:
                await self.database_port.upsert(model.to_dict())
            except ServiceException as e:
                print("[control_dock] DB Exception : ", e)

            # 3) 요청 검사
            model.check_variables()

            # 4) 요청 전송
            result = await rb_amr_sdk.control.control_dock(
                robot_model=self.robot_model,
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

    async def control_undock(self):
        pass

    async def control_dockStop(self):
        pass

    async def control_chargeTrigger(self):
        pass

    async def control_get_safetyField(self):
        pass

    async def control_set_safetyField(self, request:Request_Control_SafetyFieldPD):
        pass

    async def control_get_safetyFlag(self):
        pass

    async def control_set_safetyFlag(self, request:Request_Control_SafetyFlagPD):
        pass

    async def control_led(self, request:Request_Control_LEDPD):
        pass

    async def control_get_safetyIo(self):
        pass

    async def control_set_safetyIo(self, request:Request_Control_SafetyIOPD):
        pass

    async def control_get_obsbox(self):
        pass

    async def control_set_obsbox(self, request:Request_Control_ObsBoxPD):
        pass

    async def control_detect(self, request:Request_Control_DetectPD):
        pass
