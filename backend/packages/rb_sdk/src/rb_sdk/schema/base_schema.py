from typing import Any, TypedDict


class SetVariableDTO(TypedDict):
    type: str
    name: str
    init_value: Any
