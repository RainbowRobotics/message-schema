from pydantic import BaseModel


class Response_SpeedBarPD(BaseModel):
    speedbar: float | int


class Request_Change_SpeedbarPD(BaseModel):
    components: list[str]
    speedbar: float
