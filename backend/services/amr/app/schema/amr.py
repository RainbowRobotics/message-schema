
from enum import Enum


class AmrResponseStatusEnum(str, Enum):
    """
    [AMR 응답 상태값 정의]
    """
    PENDING = "pending" # 대기(accept, reject와는 별개)
    ACCEPT  = "accept"
    REJECT  = "reject"
    MOVING  = "moving"
    PAUSE   = "pause"
    CANCEL  = "cancel"
    FAIL    = "fail"
    DONE    = "done"
    UNKNOWN = "unknown"


class AmrResponseResultEnum(str, Enum):
    """
    [AMR 응답 결과값 정의]
    """
    ACCEPT = "accept"
    REJECT = "reject"
    SUCCESS = "success"
    FAIL = "fail"
