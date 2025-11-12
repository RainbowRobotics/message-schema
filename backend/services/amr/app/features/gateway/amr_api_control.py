from fastapi import APIRouter
from rb_zenoh.client import ZenohClient

amr_control_router = APIRouter(tags=["AMR 제어"])
zenoh_client = ZenohClient()

@amr_control_router.post("/test/control")
async def test_control_command(x: float = 0, y: float = 0, theta: float = 0):
    """Qt 프로그램으로 이동 명령 전송"""
    try:
        # Qt 프로그램의 slamnav/move queryable에 이동 명령 전송
        result = zenoh_client.query_one(f"slamnav/move?x={x}&y={y}&theta={theta}", timeout=5)
        
        return {
            "message": f"Qt 프로그램으로 이동 명령 전송 성공: ({x}, {y}, {theta})",
            "move_result": result.get("dict_payload", {}),
            "status": "success"
        }
    except Exception as e:
        return {
            "message": f"Qt 프로그램으로 이동 명령 전송 실패: {str(e)}",
            "status": "error"
        }
