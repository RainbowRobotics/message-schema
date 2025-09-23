from pydantic import BaseModel


class Response_CallWhoamIPD(BaseModel):
    category: str
    name: str
    model: str
    version: str
    alias: str
