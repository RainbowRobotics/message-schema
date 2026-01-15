from rb_flat_buffers.IPC.Request_PowerControl import Request_PowerControlT
from rb_flat_buffers.IPC.Request_ReferenceControl import Request_ReferenceControlT
from rb_flat_buffers.IPC.Request_ServoControl import Request_ServoControlT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK


class RBManipulateStateSDK(RBBaseSDK):
    """Rainbow Robotics Manipulate State SDK"""



    def call_powercontrol(self, *, robot_model: str, power_option: int, flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Power Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            power_option: 파워 옵션
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_PowerControlT()
        req.powerOption = power_option
        res = self.zenoh_client.query_one(f"{robot_model}/call_powercontrol", flatbuffer_req_obj=req, flatbuffer_res_T_class=Response_FunctionsT, flatbuffer_buf_size=8)

        if res["obj_payload"] is None:
            raise RuntimeError("Call Power Control failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_servocontrol(self, *, robot_model: str, servo_option: int, flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Servo Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            servo_option: 서보 옵션
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_ServoControlT()
        req.servoOption = servo_option
        res = self.zenoh_client.query_one(f"{robot_model}/call_servocontrol", flatbuffer_req_obj=req, flatbuffer_res_T_class=Response_FunctionsT, flatbuffer_buf_size=8)

        if res["obj_payload"] is None:
            raise RuntimeError("Call Servo Control failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]

    def call_referencecontrol(self, *, robot_model: str, reference_option: int, flow_manager_args: FlowManagerArgs | None = None) -> Response_FunctionsT:
        """
        [Reference Control 호출 함수]

        Args:
            robot_model: 로봇 모델명
            reference_option: 참조 옵션
            flow_manager_args: RB PFM을 쓸때 전달된 Flow Manager 인자 (done 콜백 등)
        """
        req = Request_ReferenceControlT()
        req.refcontrolOption = reference_option
        res = self.zenoh_client.query_one(f"{robot_model}/call_referencecontrol", flatbuffer_req_obj=req, flatbuffer_res_T_class=Response_FunctionsT, flatbuffer_buf_size=8)

        if res["obj_payload"] is None:
            raise RuntimeError("Call Reference Control failed: obj_payload is None")

        if flow_manager_args is not None:
            flow_manager_args.done()

        return res["obj_payload"]
