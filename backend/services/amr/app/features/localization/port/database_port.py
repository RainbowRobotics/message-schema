"""
[AMR 이동 데이터베이스 아웃풋 포트]
"""
from datetime import (
    date,
    datetime,
)
from typing import (
    Any,
    Protocol,
    runtime_checkable,
)


@runtime_checkable
class DatabasePort(Protocol):
    """
    [AMR 이동 데이터베이스 아웃풋 포트]
    """
    async def save(self, command: dict) -> None: ...
    """
    [AMR 이동 데이터베이스 저장]
    """
    async def update(self, command: dict) -> None: ...
    async def upsert(self, command: dict) -> None: ...
    async def get_log_by_id(self, id_: str) -> dict: ...
    async def get_logs(self, options: dict) -> dict: ...
    async def delete_logs(self, options: dict) -> dict: ...
    async def archive_logs(self, cutoff_utc: datetime | date, dry_run: bool) -> dict: ...
    async def export_logs(self,
        start_dt: datetime | date,
        end_dt: datetime | date,
        filters: dict[str, Any],
        filename: str | None,
        search_text: str | None,
        fields: dict[str, Any] | None,
        sort: str | None,
        order: str | None) -> dict: ...
