from pydantic import BaseModel


class Response_ReturnValuePD(BaseModel):
    returnValue: int


class NJointfPD(BaseModel):
    f: list[float]


class NInputfPD(BaseModel):
    f: list[float]


class NCarrefPD(BaseModel):
    f: list[float]


class NDInuPD(BaseModel):
    u: list[int]


class NDOutuPD(BaseModel):
    u: list[int]


class NAinPfPD(BaseModel):
    f: list[float]


class NAOutfPD(BaseModel):
    f: list[float]


class Vec3fPD(BaseModel):
    v: list[float]


class Mat3fPD(BaseModel):
    m: list[float]
