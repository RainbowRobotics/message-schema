from datetime import UTC, datetime, timedelta
from typing import Annotated

from app.features.log.log_schema import LogItem, Response_LogCntPD, Response_LogListPD
from fastapi import APIRouter, Depends
from fastapi.responses import JSONResponse
from rb_database import AsyncIOMotorDatabase, get_db

from .log_module import LogService

log_service = LogService()

log_router = APIRouter(tags=["Log"])

Db = Annotated[AsyncIOMotorDatabase, Depends(get_db)]

# @log_router.get("/list", response_model=Response_LogListPD)
# async def list_logs(
#     db: Db,
#     # 공통 필터
#     swName: Optional[str] = Query(None),
#     level: Optional[int] = Query(None, ge=0, le=9),  # 필요시 범위 조정
#     # 무한스크롤(cursor) 파라미터
#     cursor_createdAt: Optional[str] = Query(None, description="이 값보다 작은 createdAt부터 조회 (ISO 문자열)"),
#     cursor_id: Optional[str] = Query(None, description="동일 createdAt 시 _id 타이브레이커"),
#     # 페이지네이션(page 기반). 둘 다 None이면 기본 1페이지
#     page: Optional[int] = Query(None, ge=1),
#     size: int = Query(20, ge=1, le=200),
# ):
#     """
#     - 무한스크롤: cursor_createdAt & cursor_id 가 들어오면 그 이후(next) 배치를 내려준다.
#     - 페이지네이션: page가 들어오면 page/size 기준으로 내려준다.
#     - 둘 다 없으면 첫 페이지(size 만큼).
#     """
#     q = {}
#     if swName:
#         q["swName"] = swName
#     if level is not None:
#         q["level"] = int(level)

#     sort_spec = [("createdAt", -1), ("_id", -1)]

#     # ── Cursor 기반 (무한스크롤)
#     if cursor_createdAt and cursor_id:
#         # createdAt 문자열 비교 + 동일 createdAt일 때 _id 비교로 타이브레이크
#         q["$or"] = [
#             {"createdAt": {"$lt": cursor_createdAt}},
#             {"createdAt": cursor_createdAt, "_id": {"$lt": ObjectId(cursor_id)}},
#         ]
#         cursor = db["state_logs"].find(q).sort(sort_spec).limit(size)
#         docs = await cursor.to_list(length=size)

#         items = [LogItem.model_validate(d) for d in docs]
#         if len(items) == size:
#             last = items[-1]
#             next_cursor = {"createdAt": last.createdAt, "id": last.id}
#         else:
#             next_cursor = None

#         return {"items": items, "nextCursor": next_cursor}

#     # ── Page 기반
#     skip = 0
#     if page and page > 1:
#         skip = (page - 1) * size

#     cursor = db["state_logs"].find(q).sort(sort_spec).skip(skip).limit(size)
#     docs = await cursor.to_list(length=size)
#     items = [LogItem.model_validate(d) for d in docs]

#     # page 기반에선 nextCursor도 같이 내려주면 프론트가 쉽게 무한스크롤로 전환 가능
#     if items:
#         last = items[-1]
#         next_cursor = {"createdAt": last.createdAt, "id": last.id}
#     else:
#         next_cursor = None

#     return {"items": items, "nextCursor": next_cursor}


@log_router.get("/list", response_model=Response_LogListPD)
async def get_log_list(db: Db):
    cursor = db["state_logs"].find({}).sort([("createdAt", -1), ("_id", -1)]).limit(30)
    docs = await cursor.to_list(length=30)

    items = [LogItem.model_validate(d) for d in docs]
    return {"items": items}


@log_router.get("/error_log_count_for_24h", response_model=Response_LogCntPD)
async def error_log_count_for_24h(db: Db):
    now_utc = datetime.now(UTC)

    cutoff_iso = (now_utc - timedelta(hours=24)).isoformat()

    cnt = await db["state_logs"].count_documents({"level": 2, "createdAt": {"$gte": cutoff_iso}})

    res = {"count": cnt}

    return JSONResponse(res)
