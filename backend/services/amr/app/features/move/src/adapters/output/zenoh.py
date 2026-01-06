from rb_flat_buffers.SLAMNAV.Request_Move_Circular import (
    Request_Move_CircularT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Goal import (
    Request_Move_GoalT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Jog import (
    Request_Move_JogT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Pause import (
    Request_Move_PauseT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Resume import (
    Request_Move_ResumeT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Rotate import (
    Request_Move_RotateT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Stop import (
    Request_Move_StopT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_Target import (
    Request_Move_TargetT,
)
from rb_flat_buffers.SLAMNAV.Request_Move_XLinear import (
    Request_Move_XLinearT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Circular import (
    Response_Move_CircularT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Goal import (
    Response_Move_GoalT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Pause import (
    Response_Move_PauseT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Resume import (
    Response_Move_ResumeT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Rotate import (
    Response_Move_RotateT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Stop import (
    Response_Move_StopT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_Target import (
    Response_Move_TargetT,
)
from rb_flat_buffers.SLAMNAV.Response_Move_XLinear import Response_Move_XLinearT
from rb_utils.parser import (
    t_to_dict,
)
from rb_flat_buffers.SLAMNAV.State_Change_Move import (
    State_Change_MoveT,
)

from app.features.gateway.amr_zenoh import (
    zenoh_client,
)
from app.features.move.src.domain.move_model import (
    MoveModel,
)
from app.features.move.src.port.slamnav_output import (
    SlamnavPort,
)


class SlamnavZenohAdapter(SlamnavPort):
    def __init__(self):
        self.name = "slamnav"

    def _to_state_change_move_t(self, dt: dict) -> State_Change_MoveT:
        """
        [State_Change_MoveT 변환]
        - dict: dict
        - State_Change_MoveT 객체 반환
        """
        return State_Change_MoveT(
            id=dt.get("id"),
            command=dt.get("command"),
            curPose=dt.get("curPose"),
            goalPose=dt.get("goalPose"),
            vel=dt.get("vel"),
            goalId=dt.get("goalId"),
            goalName=dt.get("goalName"),
            method=dt.get("method"),
            direction=dt.get("direction"),
            preset=dt.get("preset"),
            result=dt.get("result"),
            message=dt.get("message"),
            remainingDist=dt.get("remainingDist"),
            target=dt.get("target"),
            speed=dt.get("speed"),
            batPercent=dt.get("batPercent"),
        )

    async def send_move_goal(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Goal 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_GoalT 객체 생성
        req = Request_Move_GoalT()
        req.id = model.id
        req.goalId = model.goalId
        req.method = model.method
        req.preset = model.preset

        # 2) 요청 전송
        result = zenoh_client.query_one(
        "test/v1/move/goal",
        flatbuffer_req_obj=req,
        flatbuffer_res_T_class=Response_Move_GoalT,
        flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)


    async def send_move_target(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Target 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_GoalT 객체 생성
        req = Request_Move_TargetT()
        req.id = model.id
        req.x = model.x
        req.y = model.y
        req.z = model.z
        req.rz = model.rz
        req.method = model.method
        req.preset = model.preset

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/target",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_TargetT,
            flatbuffer_buf_size=100,
        )

        # 6) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_jog(self, model: MoveModel) -> None:
        """
        [Move Jog 전송]
        - model: MoveModel
        - None 반환
        """

        # 1) Request_Move_JogT 객체 생성
        req = Request_Move_JogT()
        req.vx = model.vx
        req.vy = model.vy
        req.wz = model.wz

        # 2) 요청 전송(jog는 반환 필요없으니 publish 사용)
        zenoh_client.publish(
            "test/v1/move/jog",
            payload=req,
        )

    async def send_move_stop(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Stop 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_StopT 객체 생성
        req = Request_Move_StopT()
        req.id = model.id

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/stop",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_StopT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_pause(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Pause 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_PauseT 객체 생성
        req = Request_Move_PauseT()
        req.id = model.id

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/pause",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_PauseT,
            flatbuffer_buf_size=100,
        )

        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_resume(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Resume 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_ResumeT 객체 생성
        req = Request_Move_ResumeT()
        req.id = model.id

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/resume",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_ResumeT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_linear(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Linear 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_XLinearT 객체 생성
        req = Request_Move_XLinearT()
        req.id = model.id
        req.target = model.target
        req.speed = model.speed

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/linear",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_XLinearT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_circular(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Circular 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_CircularT 객체 생성
        req = Request_Move_CircularT()
        req.id = model.id
        req.target = model.target
        req.speed = model.speed
        req.direction = model.direction

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/circular",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_CircularT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)

    async def send_move_rotate(self, model: MoveModel) -> State_Change_MoveT:
        """
        [Move Rotate 전송]
        - model: MoveModel
        - State_Change_MoveT 객체 반환
        """
        # 1) Request_Move_RotateT 객체 생성
        req = Request_Move_RotateT()
        req.id = model.id
        req.target = model.target
        req.speed = model.speed

        # 2) 요청 전송
        result = zenoh_client.query_one(
            "test/v1/move/rotate",
            flatbuffer_req_obj=req,
            flatbuffer_res_T_class=Response_Move_RotateT,
            flatbuffer_buf_size=100,
        )
        # 3) 결과 처리 및 반환
        dt = t_to_dict(result["dict_payload"])
        return self._to_state_change_move_t(dt)
