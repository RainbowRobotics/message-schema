from pydantic import BaseModel


class BaseControlResponsePD(BaseModel):
    returnValue: int
