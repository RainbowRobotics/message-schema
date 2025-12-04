import asyncio
from typing import Any, Literal

from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_Get_Core_Data import Request_Get_Core_DataT
from rb_flat_buffers.IPC.Request_Move_J import Request_Move_JT
from rb_flat_buffers.IPC.Request_Move_L import Request_Move_LT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Request_Set_Tool_List import Request_Set_Tool_ListT
from rb_flat_buffers.IPC.Request_SideDout_Bitcombination import Request_SideDout_BitcombinationT
from rb_flat_buffers.IPC.Request_SideDout_General import Request_SideDout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Pulse import Request_SideDout_PulseT
from rb_flat_buffers.IPC.Request_SideDout_Toggle import Request_SideDout_ToggleT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.Response_Get_Core_Data import Response_Get_Core_DataT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_schemas.sdk import FlowManagerArgs

from rb_sdk.base import RBBaseSDK
from rb_sdk.schema.manipulate_schema import (
    SideDoutArg,
    SideDoutBitcombinationArg,
    SideDoutPulseArg,
    SideDoutToggleArg,
)


class RBManipulateSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate SDK"""

    def get_variable(self, robot_model: str, name: str):
        req = Request_Get_Core_DataT()
        req.option = 0
        req.name = name

        res = self.zenoh_client.query_one(
            f"{robot_model}/get_core_data",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Core_DataT,
            flatbuffer_buf_size=256,
        )

        dict_res = res["dict_payload"]

        if dict_res is None or dict_res.get("valid") == 0:
            return None

        if dict_res.get("type") == 0:
            return dict_res["payloadNum"]
        elif dict_res.get("type") == 1:
            return dict_res["payloadArr"]["arr"]
        elif dict_res.get("type") == 2:
            return dict_res["payloadStr"]

        return None

    async def set_begin(self, *, robot_model: str, position: Any, is_enable: bool = True, speed_ratio: float | None = None, flow_manager_args: FlowManagerArgs | None = None):
        """메인 태스크 시작 위치 설정"""

        if not is_enable:
            if flow_manager_args is not None:
                topic, mv, obj, attachment = await self.zenoh_client.receive_one(
                        f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                    )

                if obj is not None:
                    flow_manager_args.ctx.update_local_variables({
                        "MANIPULATE_BEGIN_JOINTS": obj.get("jointQRef", {}).get("f", [])
                    })
                    flow_manager_args.ctx.update_global_variables({
                        "MANIPULATE_BEGIN_CARTES": obj.get("carteXRef", {}).get("f", [])
                    })

                flow_manager_args.done()
            return

        req = Request_Move_JT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = position

        move_input_target.tarValues = ni
        move_input_target.tarFrame = -1
        move_input_target.tarUnit = 0

        move_input_speed.spdMode = 1
        move_input_speed.spdVelPara = 60
        move_input_speed.spdAccPara = 120

        if speed_ratio is not None:
            move_input_speed.spdMode = 0
            move_input_speed.spdVelPara = speed_ratio
            move_input_speed.spdAccPara = 0.1

        req.target = move_input_target
        req.speed = move_input_speed

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_move_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
            timeout=2,
        )

        if flow_manager_args is not None:
            if res.get("dict_payload") is None:
                raise RuntimeError("Move failed")

            while True:
                try:
                    if not self._is_alive:
                        break

                    topic, mv, obj, attachment = await self.zenoh_client.receive_one(
                        f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                    )

                    if obj is not None and obj.get("motionMode") == 0:
                        flow_manager_args.ctx.update_local_variables({
                            "BEGIN_JOINTS": obj.get("jointQRef", {}).get("f", [])
                        })
                        flow_manager_args.ctx.update_global_variables({
                            "BEGIN_CARTES": obj.get("carteXRef", {}).get("f", [])
                        })
                        flow_manager_args.done()
                        break
                except asyncio.CancelledError:
                    print("CancelledError", flush=True)
                    break
                except Exception as e:
                    print("Exception >>", e, flush=True)
                    raise RuntimeError(e) from e

    async def call_smoothjog_stop(
        self,
        *,
        robot_model: str,
        stop_time: float | int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        req = Request_Move_SmoothJogStopT()
        req.stoptime = float(stop_time)

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            flow_manager_args.done()

        return res["dict_payload"]

    async def move_finish_at_stop(
        self,
        *,
        robot_model: str,
        finish_at: bool | None = None,
        stop_time: float | int | None = 0.3,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        parsed_finish_at = finish_at
        parsed_stop_time = stop_time

        if flow_manager_args is not None:
            parsed_finish_at = (
                flow_manager_args.args.get("finish_at")
                if flow_manager_args.args.get("finish_at") is not None
                else finish_at
            )
            parsed_stop_time = (
                flow_manager_args.args.get("stop_time")
                if flow_manager_args.args.get("stop_time") is not None
                else stop_time
            )

        if parsed_finish_at is None:
            raise ValueError("finish_at is required")

        if parsed_finish_at:
            if not isinstance(parsed_stop_time, float | int):
                raise ValueError("stop_time must be a number")

            await self.call_smoothjog_stop(robot_model=robot_model, stop_time=parsed_stop_time)

            if flow_manager_args is not None:
                flow_manager_args.done()

        return parsed_finish_at

    async def move_j(
        self,
        *,
        robot_model: str,
        tar_values: list[float],
        tar_frame: int,
        tar_unit: int,
        spd_mode: int = 1,
        spd_vel_para: float = 60,
        spd_acc_para: float = 120,
        finish_at: bool | None = None,
        stop_time: float | int | None = None,
        variable_name: str | None = None,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """관절 공간 이동 명령

        Args:
            robot_model: 로봇 모델명
            tar_values: 관절 각도 리스트
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            finish_at: 이동 정지 여부
            stop_time: 이동 정지까지 시간
            variable_name: 변수 이름
            flow_manager_args: Flow Manager 인자 (done 콜백 등)
        """
        try:
            # flatbuffer 요청 구성
            req = Request_Move_JT()
            move_input_target = MoveInput_TargetT()
            move_input_speed = MoveInput_SpeedT()

            ni = N_INPUT_fT()
            ni.f = tar_values

            move_input_target.tarValues = ni
            move_input_target.tarFrame = tar_frame
            move_input_target.tarUnit = tar_unit

            move_input_speed.spdMode = spd_mode
            move_input_speed.spdVelPara = spd_vel_para
            move_input_speed.spdAccPara = spd_acc_para

            req.target = move_input_target
            req.speed = move_input_speed

            # 명령 전송
            res = self.zenoh_client.query_one(
                f"{robot_model}/call_move_j",
                flatbuffer_req_obj=req,
                flatbuffer_res_T_class=Response_FunctionsT,
                flatbuffer_buf_size=256,
                timeout=2,
            )

            # 상태 구독 (flow_manager_args가 있는 경우)
            if flow_manager_args is not None:
                if res.get("dict_payload") is None:
                    raise RuntimeError("Move failed: dict_payload is None")
                
                is_break = False

                while True:
                    try:
                        if not self._is_alive:
                            break

                        topic, mv, obj, attachment = await self.zenoh_client.receive_one(
                            f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                        )


                        if finish_at is not None:
                            stop_move = await self.move_finish_at_stop(
                                robot_model=robot_model,
                                finish_at=finish_at,
                                stop_time=stop_time,
                                flow_manager_args=flow_manager_args,
                            )

                            if stop_move:
                                is_break = True

                        if obj is not None and obj.get("motionMode") == 0:
                            is_break = True

                        if is_break:
                            if variable_name is not None and obj is not None:
                                flow_manager_args.ctx.update_local_variables({
                                    variable_name: obj.get("jointQRef", {}).get("f", [])
                                })
                            
                            flow_manager_args.done()
                            break

                    except asyncio.CancelledError:
                        print("CancelledError", flush=True)
                        break
                    except Exception as e:
                        print("Exception >>", e, flush=True)
                        raise RuntimeError(e) from e

        except Exception as e:
            raise RuntimeError(f"Move failed: {e}") from e

    def _on_state_core(self, obj: dict, flow_manager_args: FlowManagerArgs):
        """State Core 콜백 처리"""
        if obj and obj.get("motionMode") == 0:
            # 모션 완료
            flow_manager_args.done()

    async def move_l(
        self,
        *,
        robot_model: str,
        tar_values: list[float],
        tar_frame: int,
        tar_unit: int,
        spd_mode: int = 1,
        spd_vel_para: float = 250,
        spd_acc_para: float = 1000,
        finish_at: bool | None = None,
        stop_time: float | int | None = None,
        variable_name: str | None = None,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """직선 공간 이동 명령

        Args:
            robot_model: 로봇 모델명
            tar_values: 직선 좌표 리스트
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            finish_at: 이동 정지 여부
            stop_time: 이동 정지까지 시간
            variable_name: 변수 이름
            flow_manager_args: Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Move_LT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = tar_values

        req.target.tarValues = ni
        req.target.tarFrame = tar_frame
        req.target.tarUnit = tar_unit

        req.speed.spdMode = spd_mode
        req.speed.spdVelPara = spd_vel_para
        req.speed.spdAccPara = spd_acc_para

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_move_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            if res.get("dict_payload") is None:
                raise RuntimeError("Move failed: dict_payload is None")
            
            is_break = False
            
            while True:
                try:
                    if not self._is_alive:
                        break

                    topic, mv, obj, attachment = await self.zenoh_client.receive_one(
                        f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                    )
                    
                    if finish_at is not None:
                        stop_move = await self.move_finish_at_stop(
                            robot_model=robot_model,
                            finish_at=finish_at,
                            stop_time=stop_time,
                            flow_manager_args=flow_manager_args,
                        )

                        if stop_move:
                            is_break = True

                    if obj is not None and obj.get("motionMode") == 0:
                        is_break = True

                    if is_break:
                        if variable_name is not None and obj is not None:
                            flow_manager_args.ctx.update_local_variables({
                                variable_name: obj.get("carteXRef", {}).get("f", [])
                            })

                        flow_manager_args.done()
                        break
                except asyncio.CancelledError:
                    print("CancelledError", flush=True)
                    break
                except Exception as e:
                    print("Exception >>", e, flush=True)
                    raise RuntimeError(e) from e

        return res["dict_payload"]

    async def call_move_jb(
        self,
        *,
        robot_model: str,
        pnt_para: int,
        pnt_type: int,
        tar_frame: int,
        tar_unit: int,
        tar_values: list[float],
        spd_mode: int,
        spd_vel_para: float,
        spd_acc_para: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        pass

    async def call_move_lb(
        self,
        *,
        robot_model: str,
        pnt_para: int,
        pnt_type: int,
        tar_frame: int,
        tar_unit: int,
        tar_values: list[float],
        spd_mode: int,
        spd_vel_para: float,
        spd_acc_para: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        pass

    async def point(
        self,
        *,
        robot_model: str,
        move_type: Literal["J", "L", "JB", "LB"],
        pnt_para: int | None = None,
        pnt_type: int | None = None,
        tar_frame: int,
        tar_unit: int,
        tar_values: list[float],
        spd_mode: int,
        spd_vel_para: float,
        spd_acc_para: float,
        tcp_num: int = -1,
        finish_at: bool | None = None,
        stop_time: float | int | None = None,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """협동로봇 이동할 point 설정

        Args:
            robot_model: 로봇 모델명
            move_type: 이동 타입 (J: Joint Space, L: Linear Space, JB: Joint Block, LB: Linear Block)
            pnt_para: 포인트타입 (0: 직선포인트, 1: %기반 블랜드포인트, 2: 거리기반 블랜드포인트)
            pnt_type: 블랜드 파라미터 (%기반 블랜드포인트나 거리기반 블랜드포인트을 위한 파라미터)
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            tar_values: 이동 좌표 리스트
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            tcp_num: TCP 번호 (-1: 기본값)
            finish_at: 이동 정지 여부
            stop_time: 이동 정지까지 시간
            flow_manager_args: Flow Manager 인자 (done 콜백 등)
        """

        if tcp_num != -1:
            self.set_toolist_num(robot_model=robot_model, tool_num=tcp_num)

        if move_type == "J":
            await self.move_j(
                robot_model=robot_model,
                tar_values=tar_values,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                finish_at=finish_at,
                stop_time=stop_time,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "L":
            await self.move_l(
                robot_model=robot_model,
                tar_values=tar_values,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                finish_at=finish_at,
                stop_time=stop_time,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "JB":
            if pnt_para is None or pnt_type is None:
                raise ValueError("pnt_para and pnt_type are required for JB move")

            await self.call_move_jb(
                robot_model=robot_model,
                pnt_para=pnt_para,
                pnt_type=pnt_type,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                tar_values=tar_values,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == "LB":
            if pnt_para is None or pnt_type is None:
                raise ValueError("pnt_para and pnt_type are required for LB move")

            await self.call_move_lb(
                robot_model=robot_model,
                pnt_para=pnt_para,
                pnt_type=pnt_type,
                tar_frame=tar_frame,
                tar_unit=tar_unit,
                tar_values=tar_values,
                spd_mode=spd_mode,
                spd_vel_para=spd_vel_para,
                spd_acc_para=spd_acc_para,
                flow_manager_args=flow_manager_args,
            )

    def set_toolist_num(
        self, *, robot_model: str, tool_num: int, flow_manager_args: FlowManagerArgs | None = None
    ):
        """TCP 번호 설정

        Args:
            robot_model: 로봇 모델명
            tool_num: TCP 번호
        """
        req = Request_Set_Tool_ListT()
        req.targetToolNum = tool_num

        res = self.zenoh_client.query_one(
            f"{robot_model}/set_toollist_num",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            flow_manager_args.done()

        return res["dict_payload"]

    def call_side_dout(
        self,
        *,
        robot_model: str,
        port_num: int,
        desired_out: bool,
        flow_manager_args: FlowManagerArgs | None = None,
        ):
        """Side Digital Out 호출"""

        req = Request_SideDout_GeneralT()
        req.portNum = port_num
        req.desiredOut = desired_out

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            if res.get("dict_payload") is None:
                raise RuntimeError("Side Digital Out failed: dict_payload is None")

            flow_manager_args.done()

        return res["dict_payload"]

    def call_side_dout_toggle(
        self,
        *,
        robot_model: str,
        port_num: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Side Digital Out Toggle 호출"""

        req = Request_SideDout_ToggleT()
        req.portNum = port_num
        
        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout_toggle",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            flow_manager_args.done()

        return res["dict_payload"]

    def call_side_dout_bitcombination(
        self,
        *,
        robot_model: str,
        port_start: int,
        port_end: int,
        desired_value: int,
        direction_option: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Side Digital Out Bitcombination 호출"""

        req = Request_SideDout_BitcombinationT()
        req.portStart = port_start
        req.portEnd = port_end
        req.desiredValue = desired_value
        req.directionOption = direction_option

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout_bitcombination",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=16,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            flow_manager_args.done()

        return res["dict_payload"]

    def call_side_dout_pulse(
        self,
        *,
        robot_model: str,
        port_num: int,
        block_mode: int,
        direction: int,
        time_1: int,
        time_2: int,
        time_3: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Side Digital Out Pulse 호출"""

        req = Request_SideDout_PulseT()
        req.portNum = port_num
        req.blockMode = block_mode
        req.direction = direction
        req.time1 = time_1
        req.time2 = time_2
        req.time3 = time_3

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout_pulse",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=24,
        )

        if flow_manager_args is not None and res.get("dict_payload"):
            flow_manager_args.done()

        return res["dict_payload"]

    def call_multiple_side_dout(
        self,
        *,
        robot_model: str,
        side_dout_args: list[SideDoutArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Multiple Side Digital Out 호출"""

        result: list[dict] = []

        for req in side_dout_args:
            res = self.call_side_dout(
                robot_model=robot_model,
                port_num=req["port_num"],
                desired_out=req["desired_out"],
                flow_manager_args=flow_manager_args,
            )

            result.append(res)

        if flow_manager_args is not None and len(result) == len(side_dout_args):
            flow_manager_args.done()

        if len(result) != len(side_dout_args):
            raise RuntimeError(
                "Multiple Side Digital Out failed: "
                "result length is not equal to side_dout_args length"
            )

        return result

    def call_multiple_side_dout_toggle(
        self,
        *,
        robot_model: str,
        side_dout_args: list[SideDoutToggleArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Multiple Side Digital Out Toggle 호출"""

        result: list[dict] = []

        for req in side_dout_args:
            res = self.call_side_dout_toggle(
                robot_model=robot_model,
                port_num=req["port_num"],
                flow_manager_args=flow_manager_args,
            )

            result.append(res)

        if flow_manager_args is not None and len(result) == len(side_dout_args):
            flow_manager_args.done()

        if len(result) != len(side_dout_args):
            raise RuntimeError(
                "Multiple Side Digital Out Toggle failed: "
                "result length is not equal to side_dout_args length"
            )

        return result

    def call_multiple_side_dout_bitcombination(
        self,
        *,
        robot_model: str,
        side_dout_args: list[SideDoutBitcombinationArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Multiple Side Digital Out Bitcombination 호출"""

        result: list[dict] = []

        for req in side_dout_args:
            res = self.call_side_dout_bitcombination(
                robot_model=robot_model,
                port_start=req["port_start"],
                port_end=req["port_end"],
                desired_value=req["desired_value"],
                direction_option=req["direction_option"],
                flow_manager_args=flow_manager_args,
            )

            result.append(res)

        if flow_manager_args is not None and len(result) == len(side_dout_args):
            flow_manager_args.done()

        if len(result) != len(side_dout_args):
            raise RuntimeError(
                "Multiple Side Digital Out Bitcombination failed: "
                "result length is not equal to side_dout_args length"
            )

        return result

    def call_multiple_side_dout_pulse(
        self,
        *,
        robot_model: str,
        side_dout_args: list[SideDoutPulseArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ):
        """Multiple Side Digital Out Pulse 호출"""

        result: list[dict] = []

        for req in side_dout_args:
            res = self.call_side_dout_pulse(
                robot_model=robot_model,
                port_num=req["port_num"],
                block_mode=req["block_mode"],
                direction=req["direction"],
                time_1=req["time_1"],
                time_2=req["time_2"],
                time_3=req["time_3"],
                flow_manager_args=flow_manager_args,
            )

            result.append(res)

        if flow_manager_args is not None and len(result) == len(side_dout_args):
            flow_manager_args.done()

        if len(result) != len(side_dout_args):
            raise RuntimeError(
                "Multiple Side Digital Out Pulse failed: "
                "result length is not equal to side_dout_args length"
            )

        return result
