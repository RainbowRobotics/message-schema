from app.features.control.schema.control_api import (
    Request_Control_DetectPD,
    Request_Control_LEDPD,
    Request_Control_ObsBoxPD,
    Request_Control_OnOffPD,
    Request_Control_SafetyFieldPD,
    Request_Control_SafetyFlagPD,
    Request_Control_SafetyIOPD,
)


class AmrControlService:
    def __init__(self):
        pass

    async def control_onoff(self, request:Request_Control_OnOffPD):
        pass

    async def control_dock(self):
        raise NotImplementedError("control_dock is not implemented")

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
