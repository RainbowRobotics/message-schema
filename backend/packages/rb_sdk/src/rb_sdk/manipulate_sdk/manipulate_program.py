""" Rainbow Robotics Manipulate Program SDK """
import asyncio
import time as time_module
from typing import Literal

from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_MotionResume import Request_MotionResumeT
from rb_flat_buffers.IPC.Request_ProgramAfter import Request_ProgramAfterT
from rb_flat_buffers.IPC.Request_ProgramBefore import Request_ProgramBeforeT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.Response_Get_Absolute_Value import Response_Get_Absolute_ValueT
from rb_flat_buffers.IPC.Response_Get_Relative_Value import Response_Get_Relative_ValueT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_modules.log import rb_log
from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import safe_eval_expr
from rb_utils.parser import to_json
from rb_zenoh.exeption import ZenohNoReply, ZenohReplyError

from rb_sdk.manipulate_sdk.manipulate_get_data import RBManipulateGetDataSDK

from ..base import RBBaseSDK
from .manipulate_io import RBManipulateIOSDK
from .manipulate_smbc import RBManipulateSMBCSdk
from .schema.manipulate_io_schema import FlangeDoutArg
from .schema.manipulate_move_schema import MoveInputTargetSchema
from .schema.manipulate_program_schema import (
    DigitalInputConditionSchema,
    InterfaceModbusClientPayloadSchema,
)

rb_manipulate_get_data_sdk = RBManipulateGetDataSDK()
rb_manipulate_io_sdk = RBManipulateIOSDK()
rb_manipulate_smbc_sdk = RBManipulateSMBCSdk()

class RBManipulateProgramSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Program SDK"""

    def call_resume(self, *, robot_model: str) -> Response_FunctionsT:
        """
        [협동로봇 Resume 호출 함수]
        """

        req = Request_MotionResumeT()

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )


        if res["obj_payload"] is None:
            raise RuntimeError("Call Resume failed: obj_payload is None")


        return res["obj_payload"]

    def call_pause(self, *, robot_model: str) -> Response_FunctionsT:
        """
        [협동로봇 Pause 호출 함수]
        """

        req = Request_MotionPauseT()
        res = self.zenoh_client.query_one(
            f"{robot_model}/call_pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Pause failed: obj_payload is None")

        return res["obj_payload"]

    def call_halt(self, *, robot_model: str) -> Response_FunctionsT:
        """
        [협동로봇 Halt 호출 함수]
        """

        req = Request_MotionHaltT()
        res = self.zenoh_client.query_one(
            f"{robot_model}/call_halt",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=2,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Halt failed: obj_payload is None")

        return res["obj_payload"]

    def call_program_before(self, *, robot_model: str, option: int) -> Response_FunctionsT:
        """
        [Program 시작 전 호출 함수]

        Args:
            robot_model: 로봇 모델명
            option: 프로그램 옵션
        """

        try:
            req = Request_ProgramBeforeT()
            req.option = option

            res = self.zenoh_client.query_one(
                f"{robot_model}/call_program_before",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=2,
            )

            if res["obj_payload"] is None:
                raise RuntimeError("Call Program Before failed: obj_payload is None")

            return res["obj_payload"]
        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.error(f"Call Program Before failed: {e}")
        except Exception as e:
            raise e

    def call_program_after(self, *, robot_model: str, option: int) -> Response_FunctionsT:
        """
        [Program 시작 후 호출 함수]

        Args:
            robot_model: 로봇 모델명
            option: 프로그램 옵션
        """
        try:

            req = Request_ProgramAfterT()
            req.option = option

            res = self.zenoh_client.query_one(
                f"{robot_model}/call_program_after",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=2,
            )

            if res["obj_payload"] is None:
                raise RuntimeError("Call Program After failed: obj_payload is None")

            return res["obj_payload"]
        except (ZenohNoReply, ZenohReplyError) as e:
            rb_log.error(f"Call Program After failed: {e}")
        except Exception as e:
            raise e

    async def set_begin(self, *, robot_model: str, position: list[float | int] | None = None, is_enable: bool = True, flow_manager_args: FlowManagerArgs | None = None):
        """
        [메인 태스크 시작 위치 설정 함수]

        Args:
            robot_model: 로봇 모델명
            position: 시작 위치
            is_enable: 시작 위치 설정 여부
            speed_ratio: 속도 비율
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        if is_enable:
            if flow_manager_args is not None:
                try:
                    _, _, obj, _ = await self.zenoh_client.receive_one(
                            f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                        )

                    if obj is not None:
                        flow_manager_args.ctx.update_local_variables({
                            "MANIPULATE_BEGIN_JOINTS": obj.get("jointQRef", {}).get("f", [0,0,0,0,0,0,0])
                        })
                        flow_manager_args.ctx.update_local_variables({
                            "MANIPULATE_BEGIN_CARTES": obj.get("carteXRef", {}).get("f", [0,0,0,0,0,0,0])
                        })

                    if position is not None:
                        flow_manager_args.ctx.update_local_variables({
                            "MANIPULATE_BEGIN_JOINTS": position
                        })


                except Exception:
                    flow_manager_args.ctx.update_local_variables({
                        "MANIPULATE_BEGIN_JOINTS": [0,0,0,0,0,0,0]
                    })
                    flow_manager_args.ctx.update_global_variables({
                        "MANIPULATE_BEGIN_CARTES": [0,0,0,0,0,0,0]
                    })

                    return
                finally:
                    flow_manager_args.done()
            return
        else:
            if flow_manager_args is not None:
                flow_manager_args.done()

    def alarm_or_halt(
        self,
        *,
        robot_model: str,
        option: Literal["ALARM", "HALT", "FOLDER_HALT", "SUB_PROGRAM_HALT"],
        save_log: bool = False,
        pause_option: Literal["ALL", "CURRENT"] = "ALL",
        is_only_at_ui: bool = False,
        title: str,
        content: str,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """알림 또는 정지 호출"""
        stop_type = "pause"

        if (
            is_only_at_ui
            and flow_manager_args is not None
            and not flow_manager_args.ctx.is_ui_execution
        ):
            flow_manager_args.done()
            return

        if isinstance(title, int | float):
            title = str(title)
        if isinstance(content, int | float):
            content = str(content)

        if option == "ALARM":
            self.alarm(
                title=title,
                content=content,
                robot_model=robot_model
            )

            stop_type = "pause"
        elif option == "HALT":
            self.call_halt(robot_model=robot_model)

            stop_type = "stop"
        elif option == "FOLDER_HALT" and flow_manager_args is not None:
            flow_manager_args.ctx.break_folder()
            stop_type = "continue"
        elif option == "SUB_PROGRAM_HALT" and flow_manager_args is not None:
            flow_manager_args.ctx.halt_sub_task()
            stop_type = "continue"

        if save_log:
            self.log(
                content=content or title,
                robot_model=robot_model,
                level="USER"
            )

        if flow_manager_args is not None:
            if stop_type == "pause":
                if pause_option == "ALL":
                    flow_manager_args.ctx.pause_all()
                flow_manager_args.ctx.pause()
            elif stop_type == "stop":
                flow_manager_args.ctx.stop()
            flow_manager_args.ctx.check_stop()
            flow_manager_args.done()

    def debug_variables(
        self,
        *,
        robot_model: str,
        option: Literal["DIALOG", "LOG"],
        is_only_at_ui: bool = False,
        variables: list[str],
        log_content: str | None = None,
        pause_option: Literal["ALL", "CURRENT"] = "ALL",
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """변수 디버깅 호출"""
        if (
            is_only_at_ui
            and flow_manager_args is not None
            and not flow_manager_args.ctx.is_ui_execution
        ):
            flow_manager_args.done()
            return

        if flow_manager_args is None:
            return

        real_variables = [
            to_json(var)
            for var in variables
        ]

        raw_variables: list[str] = [
            var for var in flow_manager_args.args.get("variables", [])
        ]

        if len(raw_variables) == 0:
            flow_manager_args.done()
            return

        if option == "DIALOG":
            content = ""
            for index, var in enumerate(raw_variables):
                if var != real_variables[index]:
                    content += f"{var}: {real_variables[index]}\n"
                else:
                    content += f"{var}: Undefined\n"

            self.alarm(
                title="Debug Variables",
                content=content,
                robot_model=robot_model
            )

        elif option == "LOG":
            for index, var in enumerate(raw_variables):
                content = ""
                if var != real_variables[index]:
                    content = f"{var}: {real_variables[index]}"
                else:
                    content = f"{var}: Undefined"

                self.log(
                        content=content,
                        robot_model=robot_model,
                        level="USER"
                    )
            if log_content is not None:
                self.log(
                    content=log_content,
                    robot_model=robot_model,
                    level="USER"
                )

        if flow_manager_args is not None:
            if pause_option == "ALL":
                flow_manager_args.ctx.pause_all()

            flow_manager_args.ctx.pause()
            flow_manager_args.ctx.check_stop()
            flow_manager_args.done()

    async def manipulate_wait(
        self,
        *,
        robot_model: str,
        wait_type: Literal["TIME", "HOLDING", "EXIT"] = "TIME",
        mode: Literal["GENERAL", "DIGITAL_INPUT"] = "GENERAL",
        second: float | int | None = None,
        digital_input: DigitalInputConditionSchema | None = None,
        time_out: float | int | None = None,
        flow_manager_args: FlowManagerArgs | None = None
    ):
        """
        [Manipulate Wait 커맨드 호출 함수]

        Args:
            robot_model: 로봇 모델명
            wait_type: 대기 타입
            mode: 대기 모드
            second: 대기 시간
            digital_input: 디지털 입력 조건
            time_out: 대기 시간 초과 시간
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        if flow_manager_args is None:
            return

        now = time_module.monotonic()
        condition = flow_manager_args.args.get("condition", None)
        logic_operator = (
            (digital_input.get("logical_operator") if digital_input is not None else None)
            or "AND"
        )
        is_break = False

        if wait_type == "TIME":
            await self.wait(second=second, flow_manager_args=flow_manager_args)
            return
        elif wait_type == "HOLDING":
            if mode == "GENERAL":
                while True:
                    flow_manager_args.ctx.check_stop()

                    if not safe_eval_expr(condition, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable):
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)
            elif mode == "DIGITAL_INPUT":
                while True:
                    flow_manager_args.ctx.check_stop()

                    _, _, obj, _ = await self.zenoh_client.receive_one(f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT)

                    if logic_operator == "AND":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] != signal:
                                is_break = True

                    elif logic_operator == "OR":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] == signal:
                                is_break = False
                                break
                            else:
                                is_break = True

                    if is_break:
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)


        elif wait_type == "EXIT":
            if mode == "GENERAL":
                while True:
                    flow_manager_args.ctx.check_stop()

                    if safe_eval_expr(condition, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable):
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)
            elif mode == "DIGITAL_INPUT":
                while True:
                    flow_manager_args.ctx.check_stop()

                    _, _, obj, _ = await self.zenoh_client.receive_one(f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT)

                    if logic_operator == "AND":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] != signal:
                                is_break = False
                                break
                            else:
                                is_break = True

                    elif logic_operator == "OR":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] == signal:
                                is_break = True

                    if is_break:
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)

        if flow_manager_args is not None:
            flow_manager_args.done()

    def set_pin_point_or_joint(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        reference_target: MoveInputTargetSchema | None = None,
        pin_type: Literal["POINT", "JOINT"],
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_Get_Relative_ValueT | Response_Get_Absolute_ValueT:
        """
        [Pin Point 또는 Joint 설정 함수]

        Args:
            robot_model: 로봇 모델명
            target: 좌표 및 프레임 정보
            reference_target: 기준 좌표 및 프레임 정보
            pin_type: Pin 타입 (POINT: 포인트, JOINT: 관절)
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        if flow_manager_args is None:
            return

        variable_name = flow_manager_args.args.get("variable_name", None)

        if reference_target is not None:
            res = rb_manipulate_get_data_sdk.get_relative_value(
                robot_model=robot_model,
                relative_value=target,
                reference_value=reference_target,
                move_type=1 if pin_type == "POINT" else 0,
            )

            if res.calculatedResult != 0:
                raise RuntimeError(f"Get Relative Value failed: calculatedResult={res.calculatedResult}")
        else:
            res = rb_manipulate_get_data_sdk.get_absolute_value(
                robot_model=robot_model,
                reference_value=target,
                move_type=1 if pin_type == "POINT" else 0,
            )

            if res.calculatedResult != 0:
                raise RuntimeError(f"Get Absolute Value failed: calculatedResult={res.calculatedResult}")

        calculated_value = res.calculatedValue

        if variable_name is not None:
            flow_manager_args.ctx.update_local_variables({
                variable_name: calculated_value.tarValues.f
            })

        flow_manager_args.done()

        return res

    def tool_out(self, *, robot_model: str, tool_voltage: int, tool_dout_args: list[FlangeDoutArg], flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Tool Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            tool_voltage: 툴 전압
            tool_dout_args: 툴 디지털 아웃 인자
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        if flow_manager_args is None:
            return

        res = rb_manipulate_io_sdk.call_flange_power(
            robot_model=robot_model,
            desired_voltage=tool_voltage,
        )

        if res.returnValue != 0:
            raise RuntimeError(f"Tool Out failed: returnValue={res.returnValue}")

        res = rb_manipulate_io_sdk.call_multiple_flange_dout(
            robot_model=robot_model,
            flange_dout_args=tool_dout_args,
        )

        flow_manager_args.done()

        return res

    def interface(
            self,
            *,
            option: Literal["MODBUS_CLIENT"],
            modbus_client: InterfaceModbusClientPayloadSchema,
            flow_manager_args: FlowManagerArgs | None = None
        ):
        """
        [Interface 호출 함수]

        Args:
            option: 인터페이스 옵션 (MODBUS_CLIENT)
            modbus_client: Modbus Client 인자
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        if option == "MODBUS_CLIENT":
            if flow_manager_args is not None and not modbus_client["server_ip"]:
                flow_manager_args.done()
                return

            if modbus_client["type"] == "READ":
                if modbus_client["number_of_read"] is None:
                    raise RuntimeError("Modbus Read failed: number_of_read is required")
                if modbus_client["timeout_ms"] is None:
                    raise RuntimeError("Modbus Read failed: timeout_ms is required")

                obj_payload = rb_manipulate_smbc_sdk.modbus_read(
                    issue_core=modbus_client["issue_core"],
                    server_ip=modbus_client["server_ip"],
                    server_port=modbus_client["server_port"],
                    function_code=modbus_client["function_code"],
                    register_addr=modbus_client["register_addr"],
                    number_of_read=modbus_client["number_of_read"],
                    timeout_ms=modbus_client["timeout_ms"],
                )

                if obj_payload is None:
                    raise RuntimeError("Modbus Read failed: obj_payload is None")

                if obj_payload.result != 0:
                    raise RuntimeError(f"Modbus Read failed: result={obj_payload.result}")
            elif modbus_client["type"] == "WRITE":
                if modbus_client["payload_dlc"] is None and len(modbus_client["payload"]) > 1:
                    raise RuntimeError("Modbus Write failed: payload_dlc is required")
                if modbus_client["payload"] is None:
                    raise RuntimeError("Modbus Write failed: payload is required")
                if modbus_client["timeout_ms"] is None:
                    raise RuntimeError("Modbus Write failed: timeout_ms is required")

                obj_payload = rb_manipulate_smbc_sdk.modbus_write(
                    issue_core=modbus_client["issue_core"],
                    server_ip=modbus_client["server_ip"],
                    server_port=modbus_client["server_port"],
                    function_code=modbus_client["function_code"],
                    register_addr=modbus_client["register_addr"],
                    payload_dlc=modbus_client["payload_dlc"],
                    payload=modbus_client["payload"],
                    timeout_ms=modbus_client["timeout_ms"],
                )

                if obj_payload is None:
                    raise RuntimeError("Modbus Write failed: obj_payload is None")

                if obj_payload.result != 0:
                    raise RuntimeError(f"Modbus Write failed: result={obj_payload.result}")

                if flow_manager_args is not None and modbus_client["return_variable_name"] is not None:
                    flow_manager_args.ctx.update_local_variables({
                        modbus_client["return_variable_name"]: obj_payload.payload.i
                    })
            else:
                raise RuntimeError("Modbus Client failed: modbus_read or modbus_write is required")

        if flow_manager_args is not None:
            flow_manager_args.done()
