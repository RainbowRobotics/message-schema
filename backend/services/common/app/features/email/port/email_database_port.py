"""
[Email 데이터베이스 아웃풋 포트]
"""
from datetime import (
    date,
    datetime,
)
from typing import (
    Protocol,
    runtime_checkable,
)
from app.features.email.domain.email import (
    EmailModel,
)
@runtime_checkable
class EmailDatabasePort(Protocol):
    """
    [Email 로그 데이터베이스 아웃풋 포트]
    """
    async def save(self, command: dict) -> None: ...
    async def update(self, command: dict) -> None: ...
    async def upsert(self, command: dict) -> None: ...
    async def get_log_by_id(self, id_: str) -> dict: ...
    async def get_logs(self, options: dict) -> dict: ...
    async def delete_logs(self, options: dict) -> dict: ...
    async def archive_logs(self, cutoff_utc: datetime | date, dry_run: bool) -> dict: ...
    async def export_logs(self, options: dict) -> dict: ...
