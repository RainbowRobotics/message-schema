
from rb_flat_buffers.SLAMNAV.MoveJog import MoveJogT
from rb_flat_buffers.SLAMNAV.MovePose import MovePoseT
from rb_flat_buffers.SLAMNAV.RequestMoveCircular import RequestMoveCircularT
from rb_flat_buffers.SLAMNAV.RequestMoveGoal import RequestMoveGoalT
from rb_flat_buffers.SLAMNAV.RequestMovePause import RequestMovePauseT
from rb_flat_buffers.SLAMNAV.RequestMoveResume import RequestMoveResumeT
from rb_flat_buffers.SLAMNAV.RequestMoveRotate import RequestMoveRotateT
from rb_flat_buffers.SLAMNAV.RequestMoveStop import RequestMoveStopT
from rb_flat_buffers.SLAMNAV.RequestMoveTarget import RequestMoveTargetT
from rb_flat_buffers.SLAMNAV.RequestMoveXLinear import RequestMoveXLinearT
from rb_flat_buffers.SLAMNAV.RequestMoveYLinear import RequestMoveYLinearT
from rb_flat_buffers.SLAMNAV.ResponseMoveCircular import ResponseMoveCircularT
from rb_flat_buffers.SLAMNAV.ResponseMoveGoal import ResponseMoveGoalT
from rb_flat_buffers.SLAMNAV.ResponseMovePause import ResponseMovePauseT
from rb_flat_buffers.SLAMNAV.ResponseMoveResume import ResponseMoveResumeT
from rb_flat_buffers.SLAMNAV.ResponseMoveRotate import ResponseMoveRotateT
from rb_flat_buffers.SLAMNAV.ResponseMoveStop import ResponseMoveStopT
from rb_flat_buffers.SLAMNAV.ResponseMoveTarget import ResponseMoveTargetT
from rb_flat_buffers.SLAMNAV.ResponseMoveXLinear import ResponseMoveXLinearT
from rb_flat_buffers.SLAMNAV.ResponseMoveYLinear import ResponseMoveYLinearT

from ..base import RBBaseSDK


class RBAmrMoveSDK(RBBaseSDK):
    """Rainbow Robotics AMR Move SDK"""

    async def send_move_goal(
        self,
        robot_model: str,
        robot_id: str,
        req_id: str,
        goal_id: str | None ,
        goal_name: str | None,
        method: str = "pp",
        preset: int = 0
    ) -> ResponseMoveGoalT:
        """
        [목표 위치로 이동]
        - goal_id: 목표 노드 ID
        - goal_name: 목표 노드 이름
        - method: 이동 방법 ( "pp", "hpp" )
        - preset: 프리셋 번호 (현재는 0으로 고정)
        - ResponseMoveGoalT 객체 반환
        """

        # 1) Request_Move_GoalT 객체 생성
        req = RequestMoveGoalT()
        req.id = req_id
        req.goalId = goal_id
        req.goalName = goal_name
        req.method = method
        req.preset = preset

        # 2) 요청 전송
        print(f"=> send_move_goal: {robot_model}/{robot_id}/move/goal")
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/goal",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveGoalT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Target failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_target(self, robot_model: str, robot_id: str, req_id: str, x: float, y: float, z: float, rz: float, method: str = "pp", preset: int = 0) -> ResponseMoveTargetT:
        """
        [특정 위치로 이동]
        - x: X축 위치 [m]
        - y: Y축 위치 [m]
        - z: Z축 위치 [m]
        - rz: Z축 각도 [deg]
        - method: 이동 방법 ( "pp", "hpp" )
        - preset: 프리셋 번호 (현재는 0으로 고정)
        - ResponseMoveTargetT 객체 반환
        """

        goal_pose = MovePoseT()
        goal_pose.x = x
        goal_pose.y = y
        goal_pose.z = z
        goal_pose.rz = rz

        # 1) Request_Move_GoalT 객체 생성
        req = RequestMoveTargetT()
        req.id = req_id
        req.goalPose = goal_pose
        req.method = method
        req.preset = preset

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/target",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveTargetT,
            flatbuffer_buf_size=125,
        )

        # 6) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Jog failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_jog(self, robot_model: str, robot_id: str, vx: float, vy: float, wz: float) -> None:
        """
        [Move Jog 전송]
        - vx: X축 선속도 [m/s]
        - vy: Y축 선속도 [m/s]
        - wz: Z축 각속도 [deg/s]
        - 반환 없음
        """

        # 1) MoveJogT 객체 생성
        req = MoveJogT()
        req.vx = vx
        req.vy = vy
        req.wz = wz

        # print(f"send_move_jog : {req.vx}, {req.vy}, {req.wz}")

        # 2) 요청 전송(jog는 반환 필요없으니 publish 사용)
        self.zenoh_client.publish(
            f"amr/{robot_model}/{robot_id}/move/jog",
            flatbuffer_req_obj=req,
            flatbuffer_buf_size=125,
        )

        return None

    async def send_move_stop(self, robot_model: str, robot_id: str, req_id: str) -> ResponseMoveStopT:
        """
        [이동 중지]
        - ResponseMoveStopT 객체 반환
        """

        # 1) RequestMoveStopT 객체 생성
        req = RequestMoveStopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveStopT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Stop failed: obj_payload is None")

        print(f"=> send_move_stop: {robot_model}/{robot_id}/move/stop: {result['obj_payload']}")
        return result["obj_payload"]

    async def send_move_pause(self, robot_model: str, robot_id: str, req_id: str) -> ResponseMovePauseT:
        """
        [이동 일시정지]
        - ResponseMovePauseT 객체 반환
        """

        # 1) RequestMovePauseT 객체 생성
        req = RequestMovePauseT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMovePauseT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Pause failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_resume(self, robot_model: str, robot_id: str, req_id: str) -> ResponseMoveResumeT:
        """
        [이동 재개]
        - ResponseMoveResumeT 객체 반환
        """

        # 1) RequestMoveResumeT 객체 생성
        req = RequestMoveResumeT()
        req.id = req_id

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveResumeT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Resume failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_x_linear(self, robot_model: str, robot_id: str, req_id: str, target: float, speed: float) -> ResponseMoveXLinearT:
        """
        [X축 선 이동]
        - target: 이동 거리 [m]
        - speed: 이동 속도 [m/s]
        - ResponseMoveXLinearT 객체 반환
        """

        # 1) RequestMoveXLinearT 객체 생성
        req = RequestMoveXLinearT()
        req.id = req_id
        req.target = target
        req.speed = speed

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/xLinear",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveXLinearT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Linear failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_y_linear(self, robot_model: str, robot_id: str, req_id: str, target: float, speed: float) -> ResponseMoveYLinearT:
        """
        [Y축 선 이동]
        - target: 이동 거리 [m]
        - speed: 이동 속도 [m/s]
        - ResponseMoveYLinearT 객체 반환
        """

        # 1) RequestMoveYLinearT 객체 생성
        req = RequestMoveYLinearT()
        req.id = req_id
        req.target = target
        req.speed = speed

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/yLinear",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveYLinearT,
            flatbuffer_buf_size=125,
        )

        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Linear failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_circular(self, robot_model: str, robot_id: str, req_id: str, target: float, speed: float, direction: int) -> ResponseMoveCircularT:
        """
        [원형 이동]
        - target: 회전 각도 [deg]
        - speed: 회전 속도 [deg/s]
        - direction: 회전 방향 ( "right", "left" )
        - ResponseMoveCircularT 객체 반환
        """

        # 1) RequestMoveCircularT 객체 생성
        req = RequestMoveCircularT()
        req.id = req_id
        req.target = target
        req.speed = speed
        req.direction = direction

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/circular",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveCircularT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Circular failed: obj_payload is None")

        return result["obj_payload"]

    async def send_move_rotate(self, robot_model: str, robot_id: str, req_id: str, target: float, speed: float) -> ResponseMoveRotateT:
        """
        [회전 이동]
        - target: 회전 각도 [deg]
        - speed: 회전 속도 [deg/s]
        - ResponseMoveRotateT 객체 반환
        """

        # 1) RequestMoveRotateT 객체 생성
        req = RequestMoveRotateT()
        req.id = req_id
        req.target = target
        req.speed = speed

        # 2) 요청 전송
        result = self.zenoh_client.query_one(
            f"amr/{robot_model}/{robot_id}/move/rotate",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=ResponseMoveRotateT,
            flatbuffer_buf_size=125,
        )
        # 3) 결과 처리 및 반환
        if result["obj_payload"] is None:
            raise RuntimeError("Call Move Rotate failed: obj_payload is None")

        return result["obj_payload"]
