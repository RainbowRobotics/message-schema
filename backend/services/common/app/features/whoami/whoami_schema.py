from pydantic import BaseModel


class Response_CallWhoamIPD(BaseModel):
    category: str
    model: str
    version: str
    alias: str
