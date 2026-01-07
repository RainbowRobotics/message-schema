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
from rb_flat_buffers.SLAMNAV.State_Change_Move import State_Change_MoveT
from rb_utils.parser import t_to_dict
from rb_zenoh.client import ZenohClient
from .schema.amr_move_schema import SlamnavMovePort

class RBAmrMoveSDK(SlamnavMovePort):
    """Rainbow Robotics AMR Move SDK"""
    client: ZenohClient
    def __init__(self, client: ZenohClient):
        self.client = client

    def _to_state_change_move_t(self, dt: dict) -> State_Change_MoveT:
        """
        [State_Change_MoveT 변환]
        - dict: dict
        - State_Change_MoveT 객체 반환
        """
        return State_Change_MoveT(
            id=dt.get("id"),
            command=dt.get("command"),
            curPose=dt.get("cur_pose"),
            goalPose=dt.get("goal_pose"),
            mapName=dt.get("map_name"),
            vel=dt.get("vel"),
            goalId=dt.get("goal_id"),
            goalName=dt.get("goal_name"),
            method=dt.get("method"),
            direction=dt.get("direction"),
            preset=dt.get("preset"),
            result=dt.get("result"),
            message=dt.get("message"),
            remainingDist=dt.get("remaining_dist"),
            target=dt.get("target"),
            speed=dt.get("speed"),
            batPercent=dt.get("bat_percent"),
        )

    async def send_move_goal(self, req_id: str, goal_id: str, method: str, preset: int) -> State_Change_MoveT:
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
        result = self.client.query_one(
        "test/v1/move/goal",
        flatbuffer_req_obj=req,
        flatbuffer_res_T_class=Response_Move_GoalT,
        flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)


    async def send_move_target(self, req_id: str, x: float, y: float, z: float, rz: float, method: str, preset: int) -> State_Change_MoveT:
        """
        [Move Target 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_GoalT 객체 생성
        req = Request_Move_TargetT()
        req.id = req_id
        req.x = x
        req.y = y
        req.z = z
        req.rz = rz
        req.method = method
        req.preset = preset

        # 2) 요청 전송
        result = self.client.query_one(
            "test/v1/move/target",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_TargetT,
            flatbuffer_buf_size=100,
        )

        # 6) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_jog(self, vx: float, vy: float, wz: float) -> None:
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
        self.client.publish(
            "test/v1/move/jog",
            payload=req,
        )

    async def send_move_stop(self, req_id: str) -> State_Change_MoveT:
        """
        [Move Stop 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_StopT 객체 생성
        print("===================================================")
        req = Request_Move_StopT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            "test/v1/move/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_StopT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_pause(self, req_id: str) -> State_Change_MoveT:
        """
        [Move Pause 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_PauseT 객체 생성
        req = Request_Move_PauseT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            "test/v1/move/pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_PauseT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_resume(self, req_id: str) -> State_Change_MoveT:
        """
        [Move Resume 전송]
        - model: MoveRequestModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_ResumeT 객체 생성
        req = Request_Move_ResumeT()
        req.id = req_id

        # 2) 요청 전송
        result = self.client.query_one(
            "test/v1/move/resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_ResumeT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_linear(self, req_id: str, target: list[float], speed: float) -> State_Change_MoveT:
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
        result = self.client.query_one(
            "test/v1/move/linear",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_XLinearT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_circular(self, req_id: str, target: list[float], speed: float, direction: int) -> State_Change_MoveT:
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
        result = self.client.query_one(
            "test/v1/move/circular",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_CircularT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_rotate(self, req_id: str, target: list[float], speed: float) -> State_Change_MoveT:
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
        result = self.client.query_one(
            "test/v1/move/rotate",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_RotateT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)
