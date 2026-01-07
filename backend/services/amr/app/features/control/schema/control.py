from pydantic import BaseModel


class ControlLEDResponse(BaseModel):
    id: str
    onoff: bool
    color: str
    result: str | None = None
    message: str | None = None
