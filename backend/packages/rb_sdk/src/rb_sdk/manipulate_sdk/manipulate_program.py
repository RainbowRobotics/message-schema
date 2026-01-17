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
from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import eval_value
from rb_utils.parser import to_json

from rb_sdk.manipulate_sdk.manipulate_get_data import RBManipulateGetDataSDK

from ..base import RBBaseSDK
from .schema.manipulate_move_schema import MoveInputTargetSchema
from .schema.manipulate_program_schema import DigitalInputConditionSchema

rb_manipulate_get_data_sdk = RBManipulateGetDataSDK()

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

    def call_program_after(self, *, robot_model: str, option: int) -> Response_FunctionsT:
        """
        [Program 시작 후 호출 함수]

        Args:
            robot_model: 로봇 모델명
            option: 프로그램 옵션
        """

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
        is_break = False

        if wait_type == "TIME":
            return await self.wait(second=second, flow_manager_args=flow_manager_args)
        elif wait_type == "HOLDING":
            if mode == "GENERAL":
                while True:
                    if flow_manager_args.ctx.stop_event.is_set():
                        break

                    if not eval_value(condition, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable):
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)
            elif mode == "DIGITAL_INPUT":
                while True:
                    flow_manager_args.ctx.check_stop()

                    print(f"is_alive: {self._is_alive}, time_out: {time_out}, now: {now}, time_module.monotonic(): {time_module.monotonic()}")

                    _, _, obj, _ = await self.zenoh_client.receive_one(f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT)

                    if digital_input["logical_operator"] == "AND":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] != signal:
                                is_break = True

                    elif digital_input["logical_operator"] == "OR":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cbox_digital_input", {}).get("u", [0,0,0,0,0,0,0])[index] == signal:
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
                    if flow_manager_args.ctx.stop_event.is_set():
                        break

                    if eval_value(condition, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable):
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

                    await asyncio.sleep(0.01)
            elif mode == "DIGITAL_INPUT":
                while True:
                    if not self._is_alive:
                        break
                    _, _, obj, _ = await self.zenoh_client.receive_one(f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT)

                    print(f"obj: {obj}", flush=True)
                    if digital_input["logical_operator"] == "AND":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cboxDigitalInput", {}).get("u", [0,0,0,0,0,0,0])[index] != signal:
                                is_break = False
                                break
                            else:
                                is_break = True

                    elif digital_input["logical_operator"] == "OR":
                        for index, signal in enumerate(digital_input["signal"]):
                            if signal == -1:
                                continue
                            elif obj.get("cbox_digital_input", {}).get("u", [0,0,0,0,0,0,0])[index] == signal:
                                is_break = True

                    if is_break:
                        break

                    if time_out is not None and time_module.monotonic() - now >= time_out:
                        break

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

        print(f"calculated_value: {calculated_value.tarValues.f}, variable_name: {variable_name}", flush=True)

        if variable_name is not None:
            flow_manager_args.ctx.update_local_variables({
                variable_name: calculated_value.tarValues.f
            })

        print(f"flow_manager_args.ctx.variables: {flow_manager_args.ctx.variables}", flush=True)

        flow_manager_args.done()

        return res
