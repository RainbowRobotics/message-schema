"""
[페이지네이션 응답 타입]
"""
from typing import (
    Literal,
    TypedDict,
)


class PageInfo(TypedDict):
    """
    - mode: 페이지네이션 모드 (기본: "offset")
    - page: 페이지 번호
    - pages: 총 페이지 수
    - limit: 페이지당 아이템 수
    - total: 총 아이템 수
    - sort: 정렬 필드
    - order: 정렬 순서
    """
    mode: Literal["offset"]
    page: int
    pages: int
    limit: int
    total: int
    sort: str
    order: Literal["asc", "desc"]

class LogsResponse(TypedDict):
    """
    - items: 아이템 목록
    - pageInfo: 페이지 정보
    """
    items: list[dict]
    pageInfo: PageInfo
