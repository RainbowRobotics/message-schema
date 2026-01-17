from typing import Literal, TypedDict


class DigitalInputConditionSchema(TypedDict):
    """Wait 커맨드에 사용되는 Digital Input 조건"""
    signal: list[int]
    logical_operator: Literal["AND", "OR"]
