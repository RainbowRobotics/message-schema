from pydantic import BaseModel


class BaseControlResponsePD(BaseModel):
    return_value: int
