from pydantic import BaseModel


class MoveCommand(BaseModel):
    x: float
    y: float
    robot_id: str