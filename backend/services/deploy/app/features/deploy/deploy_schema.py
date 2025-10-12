from pydantic import BaseModel


class DeployProgressSchemaPD(BaseModel):
    sw_name: str
    ip: str
    mode: str
    tag: str
    percentage: int
    service_name: str
    result: str
