""" Rainbow Robotics Manipulate IO SDK """
from rb_flat_buffers.IPC.Request_Flange_Digital_Out import Request_Flange_Digital_OutT
from rb_flat_buffers.IPC.Request_Flange_Power import Request_Flange_PowerT
from rb_flat_buffers.IPC.Request_SideAout_General import Request_SideAout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Bitcombination import Request_SideDout_BitcombinationT
from rb_flat_buffers.IPC.Request_SideDout_General import Request_SideDout_GeneralT
from rb_flat_buffers.IPC.Request_SideDout_Pulse import Request_SideDout_PulseT
from rb_flat_buffers.IPC.Request_SideDout_Toggle import Request_SideDout_ToggleT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK
from .schema.manipulate_io_schema import (
    FlangeDoutArg,
    SideAoutArg,
    SideDoutArg,
    SideDoutBitcombinationArg,
    SideDoutPulseArg,
    SideDoutToggleArg,
)


class RBManipulateIOSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Digital 신호 In/Out이나 Analog 신호 In/Out을 호출하는 SDK"""

    def call_side_dout(
        self,
        *,
        robot_model: str,
        port_num: int,
        desired_out: bool,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT:
        """
        [Side Digital Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_num: 포트 번호
            desired_out: 원하는 출력 상태
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        req = Request_SideDout_GeneralT()
        req.portNum = port_num
        req.desiredOut = desired_out

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Side Digital Out failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_side_dout_toggle(
        self,
        *,
        robot_model: str,
        port_num: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT:
        """
        [Side Digital Out Toggle 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_num: 포트 번호
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        req = Request_SideDout_ToggleT()
        req.portNum = port_num

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_dout_toggle",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Side Digital Out Toggle failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_side_dout_bitcombination(
        self,
        *,
        robot_model: str,
        port_start: int,
        port_end: int,
        desired_value: int,
        direction_option: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT:
        """
        [Side Digital Out Bitcombination 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_start: 포트 시작 번호
            port_end: 포트 끝 번호
            desired_value: 원하는 값
            direction_option: 방향 옵션
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

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

        if res["obj_payload"] is None:
            raise RuntimeError("Side Digital Out Bitcombination failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

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
    ) -> Response_FunctionsT:
        """
        [Side Digital Out Pulse 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_num: 포트 번호
            block_mode: 블록 모드
            direction: 방향
            time_1: 시간 1
            time_2: 시간 2
            time_3: 시간 3
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

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

        if res["obj_payload"] is None:
            raise RuntimeError("Side Digital Out Pulse failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_multiple_side_dout(
        self,
        *,
        robot_model: str,
        side_dout_args: list[SideDoutArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> list[Response_FunctionsT]:
        """
        [Multiple Side Digital Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            side_dout_args: Side Digital Out 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        result: list[Response_FunctionsT] = []

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
    ) -> list[Response_FunctionsT]:
        """
        [Multiple Side Digital Out Toggle 호출 함수]

        Args:
            robot_model: 로봇 모델명
            side_dout_args: Side Digital Out Toggle 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        result: list[Response_FunctionsT] = []

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
    ) -> list[Response_FunctionsT]:
        """
        [Multiple Side Digital Out Bitcombination 호출 함수]

        Args:
            robot_model: 로봇 모델명
            side_dout_args: Side Digital Out Bitcombination 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        result: list[Response_FunctionsT] = []

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
    ) -> list[Response_FunctionsT]:
        """
        [Multiple Side Digital Out Pulse 호출 함수]

        Args:
            robot_model: 로봇 모델명
            side_dout_args: Side Digital Out Pulse 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        result: list[Response_FunctionsT] = []

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

    def call_side_aout(
        self,
        *,
        robot_model: str,
        port_num: int,
        desired_voltage: float,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_FunctionsT:
        """
        [Side Analog Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_num: 포트 번호
            desired_voltage: 원하는 전압
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        req = Request_SideAout_GeneralT()
        req.portNum = port_num
        req.desiredVoltage = desired_voltage

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_side_aout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Side Analog Out failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_multiple_side_aout(
        self,
        *,
        robot_model: str,
        side_aout_args: list[SideAoutArg],
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> list[Response_FunctionsT]:
        """
        [Multiple Side Analog Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            side_aout_args: Side Analog Out 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """

        result: list[Response_FunctionsT] = []

        for req in side_aout_args:
            res = self.call_side_aout(
                robot_model=robot_model,
                port_num=req["port_num"],
                desired_voltage=req["desired_voltage"],
                flow_manager_args=flow_manager_args,
            )

            result.append(res)

        if flow_manager_args is not None and len(result) == len(side_aout_args):
            flow_manager_args.done()

        if len(result) != len(side_aout_args):
            raise RuntimeError(
                "Multiple Side Analog Out failed: "
                "result length is not equal to side_aout_args length"
            )

        return result

    def call_flange_power(self, *, robot_model: str, desired_voltage: float, flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Flange Power 호출 함수]

        Args:
            robot_model: 로봇 모델명
            desired_voltage: 원하는 전압
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Flange_PowerT()
        req.desiredVoltage = desired_voltage

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_flange_power",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Flange Power failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_flange_dout(self, *, robot_model: str, port_num: int, desired_out: bool, flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Flange Digital Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            port_num: 포트 번호
            desired_out: 원하는 출력 상태
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_Flange_Digital_OutT()
        req.portNum = port_num
        req.desiredOut = desired_out

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_flange_dout",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=8,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Flange Digital Out failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_multiple_flange_dout(self, *, robot_model: str, flange_dout_args: list[FlangeDoutArg], flow_manager_args: FlowManagerArgs | None = None) -> list[Response_FunctionsT]:
        """
        [Multiple Flange Digital Out 호출 함수]

        Args:
            robot_model: 로봇 모델명
            flange_dout_args: Flange Digital Out 호출 인자 리스트
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        result: list[Response_FunctionsT] = []

        for req in flange_dout_args:
            res = self.call_flange_dout(
                robot_model=robot_model,
                port_num=req["port_num"],
                desired_out=req["desired_out"],
                flow_manager_args=flow_manager_args,
            )
            result.append(res)

        if flow_manager_args is not None and len(result) == len(flange_dout_args):
            flow_manager_args.done()

        if len(result) != len(flange_dout_args):
            raise RuntimeError(
                "Multiple Flange Digital Out failed: "
                "result length is not equal to flange_dout_args length"
            )

        return result
