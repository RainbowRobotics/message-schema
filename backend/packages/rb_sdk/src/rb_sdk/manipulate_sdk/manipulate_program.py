""" Rainbow Robotics Manipulate Program SDK """
from rb_flat_buffers.IPC.Request_MotionHalt import Request_MotionHaltT
from rb_flat_buffers.IPC.Request_MotionPause import Request_MotionPauseT
from rb_flat_buffers.IPC.Request_MotionResume import Request_MotionResumeT
from rb_flat_buffers.IPC.Request_ProgramAfter import Request_ProgramAfterT
from rb_flat_buffers.IPC.Request_ProgramBefore import Request_ProgramBeforeT
from rb_flat_buffers.IPC.Response_Functions import Response_FunctionsT
from rb_flat_buffers.IPC.State_Core import State_CoreT
from rb_schemas.sdk import FlowManagerArgs

from ..base import RBBaseSDK


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
