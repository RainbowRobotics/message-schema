from rb_flat_buffers.SLAMNAV.Request_Move_Circular import Request_Move_CircularT
from rb_flat_buffers.SLAMNAV.Request_Move_Goal import Request_Move_GoalT
from rb_flat_buffers.SLAMNAV.Request_Move_Jog import Request_Move_JogT
from rb_flat_buffers.SLAMNAV.Request_Move_Pause import Request_Move_PauseT
from rb_flat_buffers.SLAMNAV.Request_Move_Resume import Request_Move_ResumeT
from rb_flat_buffers.SLAMNAV.Request_Move_Rotate import Request_Move_RotateT
from rb_flat_buffers.SLAMNAV.Request_Move_Stop import Request_Move_StopT
from rb_flat_buffers.SLAMNAV.Request_Move_Target import Request_Move_TargetT
from rb_flat_buffers.SLAMNAV.Request_Move_XLinear import Request_Move_XLinearT
from rb_flat_buffers.SLAMNAV.Response_Move_Circular import Response_Move_CircularT
from rb_flat_buffers.SLAMNAV.Response_Move_Goal import Response_Move_GoalT
from rb_flat_buffers.SLAMNAV.Response_Move_Pause import Response_Move_PauseT
from rb_flat_buffers.SLAMNAV.Response_Move_Resume import Response_Move_ResumeT
from rb_flat_buffers.SLAMNAV.Response_Move_Rotate import Response_Move_RotateT
from rb_flat_buffers.SLAMNAV.Response_Move_Stop import Response_Move_StopT
from rb_flat_buffers.SLAMNAV.Response_Move_Target import Response_Move_TargetT
from rb_flat_buffers.SLAMNAV.Response_Move_XLinear import Response_Move_XLinearT

from ..base import RBBaseSDK
from .schema.amr_move_schema import SlamnavMovePort


class RBAmrMoveSDK(RBBaseSDK,SlamnavMovePort):
    """Rainbow Robotics AMR Move SDK"""

    # def _to_state_change_move_t(self, dt: dict) -> State_Change_MoveT:
    #     """
    #     [State_Change_MoveT 변환]
    #     - dict: dict
    #     - State_Change_MoveT 객체 반환
    #     """
    #     return State_Change_MoveT(
    #         id=dt.get("id"),
    #         command=dt.get("command"),
    #         curPose=dt.get("curPose") if dt.get("curPose") else dt.get("cur_pose"),
    #         goalPose=dt.get("goalPose") if dt.get("goalPose") else dt.get("goal_pose"),
    #         mapName=dt.get("mapName") if dt.get("mapName") else dt.get("map_name"),
    #         vel=dt.get("vel"),
    #         goalId=dt.get("goalId") if dt.get("goalId") else dt.get("goal_id"),
    #         goalName=dt.get("goalName") if dt.get("goalName") else dt.get("goal_name"),
    #         method=dt.get("method"),
    #         direction=dt.get("direction"),
    #         preset=dt.get("preset"),
    #         result=dt.get("result"),
    #         message=dt.get("message"),
    #         remainingDist=dt.get("reamaingDist") if dt.get("reamaingDist") else dt.get("remaining_dist"),
    #         target=dt.get("target"),
    #         speed=dt.get("speed"),
    #         batPercent=dt.get("batPercent") if dt.get("batPercent") else dt.get("bat_percent"),
    #     )

    async def send_move_goal(self, robot_model: str, req_id: str, goal_id: str, method: str, preset: int) -> Response_Move_GoalT:
        """
        [Move Goal 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_GoalT 객체 생성
        req = Request_Move_GoalT()
        req.id = req_id
        req.goalId = goal_id
        req.method = method
        req.preset = preset

        # 2) 요청 전송
        print(f"=> send_move_goal: {robot_model}/move/goal")
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/goal",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_GoalT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Target failed: obj_payload is None")

        return result["obj_payload"]


    async def send_move_target(self, robot_model: str, req_id: str, goal_pose: list[float], method: str, preset: int) -> Response_Move_TargetT:
        """
        [Move Target 전송]
        - model: MoveRequestModel
        - Response_Move_TargetT 객체 반환
        """
        # 1) Request_Move_GoalT 객체 생성
        req = Request_Move_TargetT()
        req.id = req_id
        req.goalPose = goal_pose
        req.method = method
        req.preset = preset

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/target",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_TargetT,
            flatbuffer_buf_size=125,
        )

        # 6) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Jog failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_jog(self, robot_model: str, vx: float, vy: float, wz: float) -> None:
        """
        [Move Jog 전송]
        - model: MoveRequestModel
        - None 반환
        """

        # 1) Request_Move_JogT 객체 생성
        req = Request_Move_JogT()
        req.vx = vx
        req.vy = vy
        req.wz = wz

        # 2) 요청 전송(jog는 반환 필요없으니 publish 사용)
        self.zenoh_client.publish(
            f"{robot_model}/move/jog",
            payload=req,
        )
        return None

    async def send_move_stop(self, robot_model: str, req_id: str) -> Response_Move_StopT:
        """
        [Move Stop 전송]
        - model: MoveRequestModel
        - Response_Move_StopT 객체 반환
        """
        # 1) Request_Move_StopT 객체 생성
        req = Request_Move_StopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_StopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Stop failed: obj_payload is None")

        print(f"=> send_move_stop: {robot_model}/move/stop: {result['obj_payload']}")
        return result["obj_payload"]

    async def send_move_pause(self, robot_model: str, req_id: str) -> Response_Move_PauseT:
        """
        [Move Pause 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_PauseT 객체 생성
        req = Request_Move_PauseT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_PauseT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Pause failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_resume(self, robot_model: str, req_id: str) -> Response_Move_ResumeT:
        """
        [Move Resume 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_ResumeT 객체 생성
        req = Request_Move_ResumeT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_ResumeT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Resume failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_linear(self, robot_model: str, req_id: str, target: list[float], speed: float) -> Response_Move_XLinearT:
        """
        [Move Linear 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_XLinearT 객체 생성
        req = Request_Move_XLinearT()
        req.id = req_id
        req.target = target
        req.speed = speed

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/linear",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_XLinearT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Linear failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_circular(self, robot_model: str, req_id: str, target: list[float], speed: float, direction: int) -> Response_Move_CircularT:
        """
        [Move Circular 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_CircularT 객체 생성
        req = Request_Move_CircularT()
        req.id = req_id
        req.target = target
        req.speed = speed
        req.direction = direction

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/circular",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_CircularT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Circular failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_rotate(self, robot_model: str, req_id: str, target: list[float], speed: float) -> Response_Move_RotateT:
        """
        [Move Rotate 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_RotateT 객체 생성
        req = Request_Move_RotateT()
        req.id = req_id
        req.target = target
        req.speed = speed

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"{robot_model}/move/rotate",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_RotateT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Rotate failed: obj_payload is None")

        return result["obj_payload"]
