""" Rainbow Robotics Manipulate Move SDK """
import asyncio

from rb_flat_buffers.IPC.MoveInput_Speed import MoveInput_SpeedT
from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_Move_J import Request_Move_JT
from rb_flat_buffers.IPC.Request_Move_L import Request_Move_LT
from rb_flat_buffers.IPC.Request_Move_SmoothJogJ import Request_Move_SmoothJogJT
from rb_flat_buffers.IPC.Request_Move_SmoothJogL import Request_Move_SmoothJogLT
from rb_flat_buffers.IPC.Request_Move_SmoothJogStop import Request_Move_SmoothJogStopT
from rb_flat_buffers.IPC.Request_Move_TickJogJ import Request_Move_TickJogJT
from rb_flat_buffers.IPC.Request_Move_TickJogL import Request_Move_TickJogLT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import eval_value

from ..base import RBBaseSDK
from .manipulate_get_data import RBManipulateGetDataSDK
from .schema.manipulate_move_schema import (
    MoveInputSpeedSchema,
    MoveInputTargetSchema,
)


class RBManipulateMoveSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Move SDK"""

    def __init__(self):
        super().__init__()

        self.get_data_sdk = RBManipulateGetDataSDK()

    async def _move_flow_manager_solver(self, *, robot_model: str, flow_manager_args: FlowManagerArgs | None = None):
        """
        [Move 관련 함수에서 사용되는 flow manager 처리 함수]

        Args:
            robot_model: 로봇 모델명
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        if flow_manager_args is not None:
            finish_at = flow_manager_args.args.get("finish_at", None)
            stop_time = flow_manager_args.args.get("stop_time", None)
            variable_name = flow_manager_args.args.get("variable_name", None)

            while True:
                try:
                    if not self._is_alive:
                        break

                    _, _, obj, _ = await self.zenoh_client.receive_one(
                        f"{robot_model}/state_core", flatbuffer_obj_t=State_CoreT
                    )

                    if variable_name is not None and obj is not None:
                        flow_manager_args.ctx.update_local_variables({variable_name: obj.get("carteXRef", {}).get("f", [])})

                    if finish_at is not None:
                        parsed_finish_at = eval_value(finish_at, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable)

                        if parsed_finish_at:
                            parsed_stop_time = eval_value(stop_time, variables=flow_manager_args.ctx.variables, get_global_variable=flow_manager_args.ctx.get_global_variable)

                            if not isinstance(parsed_stop_time, float | int):
                                raise RuntimeError("stop_time must be a number")

                            self.call_smoothjog_stop(
                                robot_model=robot_model,
                                stop_time=parsed_stop_time
                            )

                            flow_manager_args.done()
                            break

                    if obj is not None and obj.get("motionMode") == 0:
                        flow_manager_args.done()
                        break

                except asyncio.CancelledError:
                    break
                except Exception as e:
                    raise RuntimeError(str(e)) from e

        return None

    def call_move_j(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        speed: MoveInputSpeedSchema | None = None,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT:
        """
        [협동로봇이 관절 공간 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            target: 이동 좌표 및 프레임 정보
            speed: 이동 속도 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        if speed is None:
            speed = MoveInputSpeedSchema(
                spd_mode=1,
                spd_vel_para=60,
                spd_acc_para=120,
            )

        # flatbuffer 요청 구성
        req = Request_Move_JT()
        move_input_target = MoveInput_TargetT()
        move_input_speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        move_input_target.tarValues = ni
        move_input_target.tarFrame = target["tar_frame"]
        move_input_target.tarUnit = target["tar_unit"]

        move_input_speed.spdMode = speed.get("spd_mode", 1)
        move_input_speed.spdVelPara = speed.get("spd_vel_para", 60)
        move_input_speed.spdAccPara = speed.get("spd_acc_para", 120)

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

        if res["obj_payload"] is None:
            raise RuntimeError("Move J failed: obj_payload is None")

        print("flow_manager_args", flow_manager_args, flush=True)

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]

    def call_move_l(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        speed: MoveInputSpeedSchema | None = None,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [협동로봇이 직선 공간 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            target: 이동 좌표 및 프레임 정보
            speed: 이동 속도 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        if speed is None:
            speed = MoveInputSpeedSchema(
                spd_mode=1,
                spd_vel_para=250,
                spd_acc_para=1000,
            )

        req = Request_Move_LT()
        req.target = MoveInput_TargetT()
        req.speed = MoveInput_SpeedT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed.get("spd_mode", 1)
        req.speed.spdVelPara = speed.get("spd_vel_para", 250)
        req.speed.spdAccPara = speed.get("spd_acc_para", 1000)

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_move_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
            timeout=2,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Move L failed: obj_payload is None")

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]

    def call_move_jb(
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
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [협동로봇이 관절 블랜드 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            pnt_para: 포인트타입 (0: 직선포인트, 1: %기반 블랜드포인트, 2: 거리기반 블랜드포인트)
            pnt_type: 블랜드 파라미터 (%기반 블랜드포인트나 거리기반 블랜드포인트을 위한 파라미터)
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            tar_values: 이동 좌표 리스트
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

    def call_move_lb(
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
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [협동로봇이 직선 블랜드 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            pnt_para: 포인트타입 (0: 직선포인트, 1: %기반 블랜드포인트, 2: 거리기반 블랜드포인트)
            pnt_type: 블랜드 파라미터 (%기반 블랜드포인트나 거리기반 블랜드포인트을 위한 파라미터)
            tar_frame: 기준 좌표계 (-1: Joint Space, 0: Global Frame, 1: Tool Frame, 2: User Frame, 3: Target Frame)
            tar_unit: 단위계 옵션 (0: mm/degree, 1: meter/radian, 2: inch/degree)
            tar_values: 이동 좌표 리스트
            spd_mode: 속도 측정 설정 방식 (0: %기반 설정, 1: 절대값(물리값))
            spd_vel_para: 속도
            spd_acc_para: 가속도
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

    def call_relative_move(
        self,
        *,
        robot_model: str,
        relative_value: MoveInputTargetSchema,
        reference_value: MoveInputTargetSchema,
        move_type: int,
        speed: MoveInputSpeedSchema | None = None,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [상대 좌표에서 기준 좌표로 이동하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            relative_value: 상대 좌표 및 프레임 정보
            reference_value: 기준 좌표 및 프레임 정보
            move_type: 이동 타입 (0: move J 계열, 1: move L 계열)
            speed: 이동 속도 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        res_get_relative_value = self.get_data_sdk.get_relative_value(
            robot_model=robot_model,
            relative_value=relative_value,
            reference_value=reference_value,
            move_type=move_type,
        )

        if res_get_relative_value.calculatedResult != 0:
            raise RuntimeError(
                f"Relative move value getting failed: calculated_result={res_get_relative_value.calculatedResult}"
            )

        calculated_value = res_get_relative_value.calculatedValue

        target = MoveInputTargetSchema(
            tar_values=calculated_value.tarValues.f,
            tar_frame=calculated_value.tarFrame,
            tar_unit=calculated_value.tarUnit,
        )

        if move_type == 0:
            return self.call_move_j(
                robot_model=robot_model,
                target=target,
                speed=speed,
                flow_manager_args=flow_manager_args,
            )
        elif move_type == 1:
            return self.call_move_l(
                robot_model=robot_model,
                target=target,
                speed=speed,
                flow_manager_args=flow_manager_args,
            )

    def call_smoothjog_j(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT:
        """
        [Jog로 협동로봇의 관절 공간 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            target: 이동 좌표 및 프레임 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Move_SmoothJogJT()
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Smooth Jog J failed: obj_payload is None")

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]

    def call_smoothjog_l(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [Jog로 협동로봇의 직선 공간 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            target: 이동 좌표 및 프레임 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Move_SmoothJogLT()
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=256,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Smooth Jog L failed: obj_payload is None")

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]

    def call_smoothjog_stop(
        self,
        *,
        robot_model: str,
        stop_time: float | int,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT:
        """
        [협동로봇이 움직임을 stop_time만큼의 m/s 가속도로 정지하게 하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            stop_time: 정지 시간
            flow_manager_args: Flow Manager 인자 (done 콜백 등)
        """

        req = Request_Move_SmoothJogStopT()
        req.stoptime = float(stop_time)

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_smoothjog_stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if flow_manager_args is not None:
            flow_manager_args.done()

        if res["obj_payload"] is None:
            raise RuntimeError("Smooth Jog Stop failed: obj_payload is None")

        return res["obj_payload"]

    def call_tickjog_j(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        speed: MoveInputSpeedSchema,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [Tick 단위 만큼만 협동로봇의 관절 공간 이동을 시작하는 명령 전송 함수]

        Args:
            robot_model: 로봇 모델명
            target: 이동 좌표 및 프레임 정보
            speed: 이동 속도 정보
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Move_TickJogJT()
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed.get("spd_mode", 1)
        req.speed.spdVelPara = speed.get("spd_vel_para", 60)
        req.speed.spdAccPara = speed.get("spd_acc_para", 120)

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_tickjog_j",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Tick Jog J failed: obj_payload is None")

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]

    def call_tickjog_l(
        self,
        *,
        robot_model: str,
        target: MoveInputTargetSchema,
        speed: MoveInputSpeedSchema,
        flow_manager_args: FlowManagerArgs | None = None
    ) -> Response_FunctionsT | None:
        """
        [Tick 단위 만큼만 협동로봇의 직선 공간 이동을 시작하는 명령 전송 함수]
        """

        req = Request_Move_TickJogLT()
        req.target = MoveInput_TargetT()

        ni = N_INPUT_fT()
        ni.f = target["tar_values"]

        req.target.tarValues = ni
        req.target.tarFrame = target["tar_frame"]
        req.target.tarUnit = target["tar_unit"]

        req.speed.spdMode = speed.get("spd_mode", 1)
        req.speed.spdVelPara = speed.get("spd_vel_para", 60)
        req.speed.spdAccPara = speed.get("spd_acc_para", 120)

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_tickjog_l",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Tick Jog L failed: obj_payload is None")

        self._run_coro_blocking(
            self._move_flow_manager_solver(
                robot_model=robot_model,
                flow_manager_args=flow_manager_args
            )
        )

        return res["obj_payload"]
