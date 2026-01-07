
# from pydantic import BaseModel


# class MoveGoalResponse(BaseModel):
#   id: str
#   goal_id: str
#   method: str
#   preset: int
#   result: str | None = None
#   message: str | None = None

# class MoveTargetResponse(BaseModel):
#   id: str
#   method: str
#   goal_pose: list[float]
#   preset: int
#   result: str | None = None
#   message: str | None = None

# class MoveJogResponse(BaseModel):
#   id: str
#   vx: float
#   vy: float
#   wz: float

# class MoveStopResponse(BaseModel):
#   id: str
#   result: str | None = None
#   message: str | None = None

# class MovePauseResponse(BaseModel):
#   id: str
#   result: str | None = None
#   message: str | None = None

# class MoveResumeResponse(BaseModel):
#   id: str
#   result: str | None = None
#   message: str | None = None
