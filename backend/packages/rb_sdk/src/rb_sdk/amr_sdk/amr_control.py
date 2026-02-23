import asyncio
from pydantic import BaseModel
from rb_flat_buffers.SLAMNAV.ObsBox import ObsBoxT
from rb_flat_buffers.SLAMNAV.RequestChargeTrigger import RequestChargeTriggerT
from rb_flat_buffers.SLAMNAV.RequestDock import RequestDockT
from rb_flat_buffers.SLAMNAV.RequestDockStop import RequestDockStopT
from rb_flat_buffers.SLAMNAV.RequestGetObsBox import RequestGetObsBoxT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyField import RequestGetSafetyFieldT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyFlag import RequestGetSafetyFlagT
from rb_flat_buffers.SLAMNAV.RequestGetSafetyIo import RequestGetSafetyIoT
from rb_flat_buffers.SLAMNAV.RequestSetJog import RequestSetJogT
from rb_flat_buffers.SLAMNAV.RequestSetLed import RequestSetLedT
from rb_flat_buffers.SLAMNAV.RequestSetMotor import RequestSetMotorT
from rb_flat_buffers.SLAMNAV.RequestSetObsBox import RequestSetObsBoxT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyField import RequestSetSafetyFieldT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyFlag import RequestSetSafetyFlagT
from rb_flat_buffers.SLAMNAV.RequestSetSafetyIo import RequestSetSafetyIoT
from rb_flat_buffers.SLAMNAV.RequestStreamFrequency import RequestStreamFrequencyT
from rb_flat_buffers.SLAMNAV.RequestUndock import RequestUndockT
from rb_flat_buffers.SLAMNAV.ResponseChargeTrigger import ResponseChargeTriggerT
from rb_flat_buffers.SLAMNAV.ResponseDock import ResponseDockT
from rb_flat_buffers.SLAMNAV.ResponseDockStop import ResponseDockStopT
from rb_flat_buffers.SLAMNAV.ResponseGetObsBox import ResponseGetObsBoxT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyField import ResponseGetSafetyFieldT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyFlag import ResponseGetSafetyFlagT
from rb_flat_buffers.SLAMNAV.ResponseGetSafetyIo import ResponseGetSafetyIoT
from rb_flat_buffers.SLAMNAV.ResponseSetJog import ResponseSetJogT
from rb_flat_buffers.SLAMNAV.ResponseSetLed import ResponseSetLedT
from rb_flat_buffers.SLAMNAV.ResponseSetMotor import ResponseSetMotorT
from rb_flat_buffers.SLAMNAV.ResponseSetObsBox import ResponseSetObsBoxT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyField import ResponseSetSafetyFieldT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyFlag import ResponseSetSafetyFlagT
from rb_flat_buffers.SLAMNAV.ResponseSetSafetyIo import ResponseSetSafetyIoT
from rb_flat_buffers.SLAMNAV.ResponseStreamFrequency import ResponseStreamFrequencyT
from rb_flat_buffers.SLAMNAV.ResponseUndock import ResponseUndockT
from rb_flat_buffers.SLAMNAV.SafetyFlag import SafetyFlagT
from rb_schemas.sdk import FlowManagerArgs
from rb_flat_buffers.SLAMNAV.Status import StatusT
from rb_zenoh.exeption import ZenohNoReply

from ..base import RBBaseSDK


class SafetyFlag(BaseModel):
    name: str
    value: bool

class RBAmrControlSDK(RBBaseSDK):
    """Rainbow Robotics AMR Control SDK"""

    async def _dock_flow_manager_solver(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None):
        """
        [Dock 관련 함수에서 사용되는 flow manager 처리 함수]
        """
        if flow_manager_args is not None:
            while True:
                try:
                    print("FLOW MANAGER SOLVER TRY>>>>>>>>>", flush=True)
                    if not self._is_alive:
                        break

                    _, _, obj, _ = await self.zenoh_client.receive_one(
                        f"amr/{robot_model}/{robot_id}/status", flatbuffer_obj_t=StatusT, timeout=1
                    )

                    if obj is None:
                        continue

                    print(f"FLOW MANAGER STATUS >>>>>>>>>>> {obj.get("chargeState").get("dockId")} {req_id} {obj.get("chargeState").get("dockResult")}", flush=True)

                    if obj.get("chargeState").get("dockId") != req_id:
                        raise RuntimeError("Dock ID Mismatch")

                    if obj.get("chargeState").get("dockResult") == "success":
                        print("Dock Flow Manager SOLVER DONE", flush=True)
                        flow_manager_args.done()
                        break
                    elif obj.get("chargeState").get("dockResult") == "fail":
                        raise RuntimeError("Dock Fail")
                    elif obj.get("chargeState").get("dockResult") == "cancel":
                        raise RuntimeError("Dock Cancel")
                except ZenohNoReply as e:
                    raise RuntimeError(str(e)) from e
                except asyncio.CancelledError as e:
                    print(f"DOCK FLOW MANAGER SOLVER CANCELLED>>>>>>>>> {e}", flush=True)
                    break
                except Exception as e:
                    print(f"DOCK FLOW MANAGER SOLVER ERROR>>>>>>>>> {e}", flush=True)
                    raise RuntimeError(str(e)) from e

    async def _charge_trigger_flow_manager_solver(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None):
        """
        [Charge Trigger 관련 함수에서 사용되는 flow manager 처리 함수]
        """
        if flow_manager_args is not None:
            while True:
                try:
                    print("FLOW MANAGER SOLVER TRY>>>>>>>>>", flush=True)
                    if not self._is_alive:
                        break

                    _, _, obj, _ = await self.zenoh_client.receive_one(
                        f"amr/{robot_model}/{robot_id}/status", flatbuffer_obj_t=StatusT, timeout=1
                    )

                    if obj is None:
                        continue

                    print(f"FLOW MANAGER STATUS >>>>>>>>>>> {obj.get("chargeState").get("triggerId")} {req_id} {obj.get("chargeState").get("triggerResult")}", flush=True)

                    if obj.get("chargeState").get("triggerId") != req_id:
                        raise RuntimeError("Trigger ID Mismatch")

                    if obj.get("chargeState").get("triggerResult") == "success":
                        print("Trigger Flow Manager SOLVER DONE", flush=True)
                        flow_manager_args.done()
                        break
                    elif obj.get("chargeState").get("triggerResult") == "fail":
                        raise RuntimeError("Trigger Fail")
                    elif obj.get("chargeState").get("triggerResult") == "cancel":
                        raise RuntimeError("Trigger Cancel")
                except ZenohNoReply as e:
                    raise RuntimeError(str(e)) from e
                except asyncio.CancelledError as e:
                    print(f"TRIGGER FLOW MANAGER SOLVER CANCELLED>>>>>>>>> {e}", flush=True)
                    break
                except Exception as e:
                    print(f"TRIGGER FLOW MANAGER SOLVER ERROR>>>>>>>>> {e}", flush=True)
                    raise RuntimeError(str(e)) from e


    async def control_get_safety_field(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseGetSafetyFieldT:
        """
        [안전 필드 조회]
        - ResponseGetSafetyFieldT 객체 반환
        """

        # 1) RequestGetSafetyFieldT 객체 생성
        req = RequestGetSafetyFieldT()

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/getSafetyField",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyFieldT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety Field failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_safety_field(self, robot_model: str, robot_id: str, req_id: str, safety_field: int, flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetSafetyFieldT:
        """
        [안전 필드 설정]
        - safety_field: 안전 필드 번호
        - ResponseSetSafetyFieldT 객체 반환
        """

        # 1) RequestSetSafetyFieldT 객체 생성
        req = RequestSetSafetyFieldT()
        req.id = req_id
        req.safety_field = safety_field

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/setSafetyField",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyFieldT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety Field failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_get_safety_flag(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseGetSafetyFlagT:
        """
        [안전 플래그 조회]
        - ResponseGetSafetyFlagT 객체 반환
        """

        # 1) RequestGetSafetyFlagT 객체 생성
        req = RequestGetSafetyFlagT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/getSafetyFlag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyFlagT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Reset Safety Flag failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_safety_flag(self, robot_model: str, robot_id: str, req_id: str, reset_flag: list[SafetyFlag], flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetSafetyFlagT:
        """
        [안전 플래그 설정]
        - reset_flag: 안전 플래그 목록
          - key: 안전 플래그 이름 (string)
          - value: 안전 플래그 값 (bool)
        - ResponseSetSafetyFlagT 객체 반환
        """

        # 1) RequestSetSafetyFlagT 객체 생성
        req = RequestSetSafetyFlagT()
        req.id = req_id
        req.safetyFlag = []

        for flag in reset_flag:
            safety_flag = SafetyFlagT()
            safety_flag.name = flag.name
            safety_flag.value = flag.value
            req.safetyFlag.add(safety_flag)

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/setSafetyFlag",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyFlagT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Reset Safety Flag failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_get_safety_io(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseGetSafetyIoT:
        """
        [안전 IO 조회]
        - ResponseGetSafetyIoT 객체 반환
        """

        # 1) RequestGetSafetyIoT 객체 생성
        req = RequestGetSafetyIoT()
        req.id = req_id
        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/getSafetyIo",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetSafetyIoT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety IO failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_safety_io(self, robot_model: str, robot_id: str, req_id: str, mcu0_din: list[bool], mcu1_din: list[bool], flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetSafetyIoT:
        """
        [안전 IO 설정]
        - mcu0_din: MCU0 Digital Input (8bit)
        - mcu1_din: MCU1 Digital Input (8bit)
        - ResponseSetSafetyIoT 객체 반환
        """

        # 1) RequestSetSafetyIoT 객체 생성
        req = RequestSetSafetyIoT()
        req.id = req_id
        req.mcu0_din = mcu0_din
        req.mcu1_din = mcu1_din

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/setSafetyIo",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetSafetyIoT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Safety IO failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_dock(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseDockT:
        """
        [도킹 제어]
        - ResponseDockT 객체 반환
        """

        # 1) RequestDockT 객체 생성
        req = RequestDockT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/dock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDockT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Undock failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                self._run_coro_blocking(
                    self._dock_flow_manager_solver(
                        robot_model=robot_model,
                        robot_id=robot_id,
                        req_id=req_id,
                        flow_manager_args=flow_manager_args,
                    )
                )
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_undock(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseUndockT:
        """
        [도킹 제어]
        - ResponseUndockT 객체 반환
        """

        # 1) RequestDockT 객체 생성
        req = RequestUndockT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/undock",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseUndockT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Undock failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                self._run_coro_blocking(
                    self._dock_flow_manager_solver(
                        robot_model=robot_model,
                        robot_id=robot_id,
                        req_id=req_id,
                        flow_manager_args=flow_manager_args,
                    )
                )
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_dock_stop(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseDockStopT:
        """
        [도킹 제어]
        - ResponsedockStopT 객체 반환
        """

        # 1) RequestDockT 객체 생성
        req = RequestDockStopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/dockStop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseDockStopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Dock Stop failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_charge_trigger(self, robot_model: str, robot_id: str, req_id: str, switch: bool, flow_manager_args: FlowManagerArgs | None = None) -> ResponseChargeTriggerT:
        """
        [충전 트리거 제어]
        - switch: 충전 트리거 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseChargeTriggerT 객체 반환
        """
        # 1) RequestChargeTriggerT 객체 생성
        req = RequestChargeTriggerT()
        req.id = req_id
        req.switch = switch

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/chargeTrigger",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseChargeTriggerT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Charge Trigger failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                self._run_coro_blocking(
                    self._charge_trigger_flow_manager_solver(
                        robot_model=robot_model,
                        robot_id=robot_id,
                        req_id=req_id,
                        flow_manager_args=flow_manager_args,
                    )
                )
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_get_obs_box(self, robot_model: str, robot_id: str, req_id: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseGetObsBoxT:
        """
        [장애물 박스 조회]
        - ResponseGetObsBoxT 객체 반환
        """

        # 1) RequestGetObsBoxT 객체 생성
        req = RequestGetObsBoxT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/getObsBox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseGetObsBoxT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Obs Box failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_obs_box(self, robot_model: str, robot_id: str, req_id: str, min_x: float, min_y: float, min_z: float, max_x: float, max_y: float, max_z: float, map_range: float, flow_manager_args: FlowManagerArgs | None = None) -> ResponseGetObsBoxT:
        """
        [장애물 박스 설정]
        - min_x: 최소 X 좌표
        - min_y: 최소 Y 좌표
        - min_z: 최소 Z 좌표
        - max_x: 최대 X 좌표
        - max_y: 최대 Y 좌표
        - max_z: 최대 Z 좌표
        - map_range: 맵 범위
        - ResponseSetObsBox 객체 반환
        """

        # 1) RequestSetObsBoxT 객체 생성
        req = RequestSetObsBoxT()
        req.id = req_id
        minT = ObsBoxT()
        minT.x = min_x
        minT.y = min_y
        minT.z = min_z
        maxT = ObsBoxT()
        maxT.x = max_x
        maxT.y = max_y
        maxT.z = max_z
        req.min = minT
        req.max = maxT
        req.range = map_range

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/setObsBox",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetObsBoxT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Obs Box failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_led(self, robot_model: str, robot_id: str, req_id: str, switch: bool, color: str, flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetLedT:
        """
        [LED 제어]
        - req_id: 요청 ID
        - switch: LED 조작 모드 (True: 수동조작, False: 자동조작)
        - color: LED 색상
        - ResponseSetLedT 객체 반환
        """

        # 1) RequestSetLedT 객체 생성
        req = RequestSetLedT()
        req.id = req_id
        req.switch = switch
        req.color = color

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/led",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetLedT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control LED failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_motor(self, robot_model: str, robot_id: str, req_id: str, switch: bool, flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetMotorT:
        """
        [모터 제어]
        - req_id: 요청 ID
        - switch: 모터 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseSetMotorT 객체 반환
        """

        # 1) RequestMotorModeT 객체 생성
        req = RequestSetMotorT()
        req.id = req_id
        req.switch = switch

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/motor",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetMotorT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_set_jog(self, robot_model: str, robot_id: str, req_id: str, switch: bool, flow_manager_args: FlowManagerArgs | None = None) -> ResponseSetJogT:
        """
        [조이스틱 모드 제어]
        - req_id: 요청 ID
        - switch: 조이스틱 켜기/끄기 (True: 켜기, False: 끄기)
        - ResponseSetJogT 객체 반환
        """

        # 1) RequestSetJogT 객체 생성
        req = RequestSetJogT()
        req.id = req_id
        req.switch = switch

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/jog",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseSetJogT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    async def control_stream_frequency(self, robot_model: str, robot_id: str, req_id: str, target: str, frequency: int, flow_manager_args: FlowManagerArgs | None = None) -> ResponseStreamFrequencyT:
        """
        [스트림 주파수 제어]
        - req_id: 요청 ID
        - target: 스트림 대상 ( "camera", "lidar2d", "lidar3d", "path" )
        - frequency: 스트림 주파수 (Hz)
        - ResponseStreamFrequencyT 객체 반환
        """

        # 1) RequestStreamFrequencyT 객체 생성
        req = RequestStreamFrequencyT()
        req.id = req_id
        req.target = target
        req.frequency = frequency

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/{robot_id}/control/streamFrequency",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseStreamFrequencyT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Control Motor failed: obj_payload is None")

        if flow_manager_args is not None:
            if result["dict_payload"].get("result") == "accept":
                flow_manager_args.done()
            else:
                raise RuntimeError(result["dict_payload"].get("message"))

        return result["obj_payload"]

    # async def control_sensor_mode(self, robot_model: str, robot_id: str, req_id: str, command: str, control: bool, frequency: int) -> ResponseSensorModeT:
    #     """
    #     [센서 모드 제어]
    #     - req_id: 요청 ID
    #     - command: 센서 종류 ( "camera", "lidar2d", "lidar3d" )
    #     - control: 센서 켜기/끄기 (True: 켜기, False: 끄기)
    #     - frequency: 센서 주파수 (Hz)
    #     - ResponseSensorModeT 객체 반환
    #     """

    #     # 1) RequestSensorModeT 객체 생성
    #     req = RequestSensorModeT()
    #     req.id = req_id
    #     req.command = command
    #     req.control = control
    #     req.frequency = frequency

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/{robot_id}/control/sensor",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=ResponseSensorModeT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Control Motor failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def control_path_mode(self, robot_model: str, robot_id: str, req_id: str, control: bool, frequency: int) -> ResponsePathModeT:
    #     """
    #     [경로 모드 제어]
    #     - req_id: 요청 ID
    #     - control: 경로 전송 켜기/끄기 (True: 켜기, False: 끄기)
    #     - frequency: 경로 전송 주파수 (Hz)
    #     - ResponsePathModeT 객체 반환
    #     """

    #     # 1) RequestPathModeT 객체 생성
    #     req = RequestPathModeT()
    #     req.id = req_id
    #     req.control = control
    #     req.frequency = frequency

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/{robot_id}/control/path",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=ResponsePathModeT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Control Motor failed: obj_payload is None")

    #     return result["obj_payload"]

    # async def control_detect_marker(self, robot_model: str, robot_id: str, req_id: str, command: str, camera_number: int, camera_serial: str, marker_size: float) -> ResponseDetectMarkerT:
    #     """
    #     [마커 감지 제어]
    #     - req_id: 요청 ID
    #     - command: 감지 명령 ( "charuco", "aruco" )
    #     - camera_number: 카메라 번호
    #     - camera_serial: 카메라 시리얼 번호
    #     - marker_size: 마커 크기
    #     - ResponseDetectMarkerT 객체 반환
    #     """

    #     # 1) RequestDetectMarkerT 객체 생성
    #     req = RequestDetectMarkerT()
    #     req.id = req_id
    #     req.command = command
    #     req.number = camera_number
    #     req.serial = camera_serial
    #     req.m_size = marker_size

    #     # 2) 요청 전송
    #     result = self.zenoh_client.query_one(
    #         f"{robot_model}/{robot_id}/control/detectMarker",
    #         flatbuffer_req_obj=req,
    #         flatbuffer_res_T_class=ResponseDetectMarkerT,
    #         flatbuffer_buf_size=125,
    #     )

    #     # 3) 결과 처리 및 반환
    #     if result["obj_payload"] is None:
    #         raise RuntimeError("Call Control Motor failed: obj_payload is None")

    #     return result["obj_payload"]
