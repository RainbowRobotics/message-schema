from rb_flat_buffers.IPC.Request_MotionSpeedBar import Request_MotionSpeedBarT
from rb_flat_buffers.IPC.Request_Set_Tool_List import Request_Set_Tool_ListT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK


class RBManipulateConfigSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate Config SDK"""

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

        if res["obj_payload"] is None:
            raise RuntimeError("Set Tool List Num failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_speedbar(self, *, robot_model: str, alpha: float, flow_manager_args: FlowManagerArgs | None = None):
        """
        [Speed Bar 호출 함수]

        Args:
            robot_model: 로봇 모델명
            alpha: 스피드 바 값
        """
        req = Request_MotionSpeedBarT()
        req.alpha = alpha

        res = self.zenoh_client.query_one(
            f"{robot_model}/call_speedbar",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_FunctionsT,
            flatbuffer_buf_size=32,
        )

        if res["obj_payload"] is None:
            raise RuntimeError("Call Speed Bar failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]
