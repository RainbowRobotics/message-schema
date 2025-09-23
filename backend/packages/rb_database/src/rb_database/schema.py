from typing import Annotated

from bson import ObjectId
from pydantic import BeforeValidator


def objectid_to_str(v):
    if isinstance(v, ObjectId):
        return str(v)
    return str(ObjectId(str(v)))


PyObjectId = Annotated[str, BeforeValidator(objectid_to_str)]
