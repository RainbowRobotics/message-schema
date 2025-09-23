from pydantic import BaseModel


class Response_ReturnValuePD(BaseModel):
    return_value: int
