from fastapi import APIRouter

from app.usecases.move.service import execute_move

# from app.usecases.move.schema import MoveCommand

router = APIRouter()

@router.get("/move")
def move(x: float, y: float, z: float):
    return execute_move(x, y, z)