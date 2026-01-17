""" Rainbow Robotics Manipulate Get Data SDK """

from rb_flat_buffers.IPC.MoveInput_Target import MoveInput_TargetT
from rb_flat_buffers.IPC.N_INPUT_f import N_INPUT_fT
from rb_flat_buffers.IPC.Request_Get_Absolute_Value import Request_Get_Absolute_ValueT
from rb_flat_buffers.IPC.Request_Get_Core_Data import Request_Get_Core_DataT
from rb_flat_buffers.IPC.Request_Get_Relative_Value import Request_Get_Relative_ValueT
from rb_flat_buffers.IPC.Response_Get_Absolute_Value import Response_Get_Absolute_ValueT
from rb_flat_buffers.IPC.Response_Get_Core_Data import Response_Get_Core_DataT
from rb_flat_buffers.IPC.Response_Get_Relative_Value import Response_Get_Relative_ValueT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK
from .schema.manipulate_move_schema import MoveInputTargetSchema


class RBManipulateGetDataSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Get Data SDK"""

    def get_variable(self, robot_model: str, name: str) -> float | list[float] | str | None:
        """
        [변수 조회 함수]

        Args:
            robot_model: 로봇 모델명
            name: 변수 이름
        """
        req = Request_Get_Core_DataT()
        req.option = 0
        req.name = name

        res = self.zenoh_client.query_one(
            f"{robot_model}/get_core_data",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Get_Core_DataT,
            flatbuffer_buf_size=256,
        )

        obj_res = res["obj_payload"]

        if obj_res is None or obj_res.valid == 0:
            return None

        if obj_res.type == 0:
            return obj_res.payloadNum
        elif obj_res.type == 1:
            return obj_res.payloadArr.arr
        elif obj_res.type == 2:
            return obj_res.payloadStr

        return None


    def get_relative_value(
        self,
        *,
        robot_model: str,
        relative_value: MoveInputTargetSchema,
        reference_value: MoveInputTargetSchema,
        move_type: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_Get_Relative_ValueT:
        """
        [reference_value 좌표에서 relative_value 값으로 움직이고자 할때 kinematics 계산 값을 반환하는 함수]

        Args:
            robot_model: 로봇 모델명
            relative_value: 상대 좌표 및 프레임 정보
            reference_value: 기준 좌표 및 프레임 정보
            move_type: 이동 타입 (0: move J 계열, 1: move L 계열)
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req_relative_move_input_target = MoveInput_TargetT()
        req_relative_move_input_target.tarValues = N_INPUT_fT()
        req_relative_move_input_target.tarValues.f = relative_value["tar_values"]
        req_relative_move_input_target.tarFrame = relative_value["tar_frame"]
        req_relative_move_input_target.tarUnit = relative_value["tar_unit"]

        req_reference_move_input_target = MoveInput_TargetT()
        req_reference_move_input_target.tarValues = N_INPUT_fT()
        req_reference_move_input_target.tarValues.f = reference_value["tar_values"][:7]
        req_reference_move_input_target.tarFrame = reference_value["tar_frame"]
        req_reference_move_input_target.tarUnit = reference_value["tar_unit"]

        req_get_relative_value = Request_Get_Relative_ValueT()
        req_get_relative_value.relativeValue = req_relative_move_input_target
        req_get_relative_value.referenceValue = req_reference_move_input_target
        req_get_relative_value.moveType = move_type

        res_get_relative_value = self.zenoh_client.query_one(
            f"{robot_model}/get_relative_value",
            flatbuffer_req_obj=req_get_relative_value,
            flatbuffer_res_T_class=Response_Get_Relative_ValueT,
            flatbuffer_buf_size=256,
        )

        res_get_relative_value_obj = res_get_relative_value.get("obj_payload")

        if res_get_relative_value_obj is None:
            raise RuntimeError("Get Relative Value failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res_get_relative_value_obj

    def get_absolute_value(
        self,
        *,
        robot_model: str,
        reference_value: MoveInputTargetSchema,
        move_type: int,
        flow_manager_args: FlowManagerArgs | None = None,
    ) -> Response_Get_Absolute_ValueT:
        """
        [reference_value 좌표로 절대 좌표로 움직이고자 할때 kinematics 계산 값을 반환하는 함수]

        Args:
            robot_model: 로봇 모델명
            reference_value: 기준 좌표 및 프레임 정보
            move_type: 이동 타입 (0: move J or pin joint 계열, 1: move L or pin point 계열)
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req_reference_move_input_target = MoveInput_TargetT()
        req_reference_move_input_target.tarValues = N_INPUT_fT()
        req_reference_move_input_target.tarValues.f = reference_value["tar_values"][:7]
        req_reference_move_input_target.tarFrame = reference_value["tar_frame"]
        req_reference_move_input_target.tarUnit = reference_value["tar_unit"]

        req_get_absolute_value = Request_Get_Absolute_ValueT()
        req_get_absolute_value.referenceValue = req_reference_move_input_target
        req_get_absolute_value.moveType = move_type

        res_get_absolute_value = self.zenoh_client.query_one(
            f"{robot_model}/get_absolute_value",
            flatbuffer_req_obj=req_get_absolute_value,
            flatbuffer_res_T_class=Response_Get_Absolute_ValueT,
            flatbuffer_buf_size=256,
        )

        res_get_absolute_value_obj = res_get_absolute_value.get("obj_payload")

        if res_get_absolute_value_obj is None:
            raise RuntimeError("Get Absolute Value failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res_get_absolute_value_obj
