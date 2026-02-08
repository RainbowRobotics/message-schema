import asyncio
import csv
import io
from datetime import (
    UTC,
    datetime,
    timedelta,
)
from typing import Any

from app.socket.socket_client import (
    socket_client,
)
from fastapi.responses import (
    StreamingResponse,
)
from pymongo import DESCENDING
from rb_database.mongo_db import (
    MongoDB,
    get_db,
)
from rb_database.utils import (
    make_check_date_range_query,
    make_check_include_query,
    make_check_search_text_query,
)
from rb_modules.log import (
    rb_log,
)
from rb_modules.service import (
    BaseService,
)
from rb_utils.asyncio_helper import (
    fire_and_log,
)
from rb_utils.file import (
    content_disposition_header,
    sanitize_filename,
)
from rb_utils.parser import (
    t_to_dict,
)

from .log_schema import (
    LogItem,
    Request_LogListParamsPD,
)


class LogService(BaseService):
    singleton = True

    def __init__(self):
        if getattr(self, "_service_initialized", False):
            return

        super().__init__()
        self._level_to_name = {
            0: "info",
            1: "warning",
            2: "error",
            3: "user",
            4: "debug",
            5: "general",
        }
        self._name_to_level = {v: k for k, v in self._level_to_name.items()}

        #메모리 카운터
        self._error_count = 0
        self._warning_count = 0
        self._count_lock = asyncio.Lock()
        self._initialized = False  #초기화 플래그
        self._service_initialized = True

    async def _ensure_initialized(self, db: MongoDB):
        """첫 호출 시 자동으로 초기화 (lazy initialization)"""
        if self._initialized:
            return

        async with self._count_lock:
            # Double-check pattern
            if self._initialized:
                return

            try:
                now_dt = datetime.now(UTC)
                cutoff_dt = now_dt - timedelta(hours=24)

                self._error_count = await db["state_logs"].count_documents(
                    {"level": {"$in": [2, "error"]}, "createdAtDt": {"$gte": cutoff_dt}}
                )

                self._warning_count = await db["state_logs"].count_documents(
                    {"level": {"$in": [1, "warning"]}, "createdAtDt": {"$gte": cutoff_dt}}
                )

                self._initialized = True

                rb_log.info(f"📊 Log counters initialized: error={self._error_count}, warning={self._warning_count}")

            except Exception as e:
                rb_log.error(f"Failed to initialize counters: {e}")

    async def start_count_updater(self):
        """백그라운드: 1분마다 DB와 동기화 (startup에서 호출)"""
        db = await get_db()

        while True:
            try:
                await asyncio.sleep(60)

                #초기화되지 않았다면 먼저 초기화
                await self._ensure_initialized(db)

                now_dt = datetime.now(UTC)
                cutoff_dt = now_dt - timedelta(hours=24)

                error_cnt = await db["state_logs"].count_documents(
                    {"level": {"$in": [2, "error"]}, "createdAtDt": {"$gte": cutoff_dt}}
                )

                warning_cnt = await db["state_logs"].count_documents(
                    {"level": {"$in": [1, "warning"]}, "createdAtDt": {"$gte": cutoff_dt}}
                )

                async with self._count_lock:
                    if self._error_count != error_cnt:
                        rb_log.debug(f"📊 Error count synced: {self._error_count} → {error_cnt}")
                        self._error_count = error_cnt
                        await socket_client.emit("error_log_count_for_24h", {"count": error_cnt})

                    if self._warning_count != warning_cnt:
                        rb_log.debug(f"📊 Warning count synced: {self._warning_count} → {warning_cnt}")
                        self._warning_count = warning_cnt
                        await socket_client.emit("warning_log_count_for_24h", {"count": warning_cnt})

            except Exception as e:
                rb_log.error(f"count updater error: {e}")

    async def get_log_list(
        self,
        *,
        db: MongoDB,
        params: Request_LogListParamsPD,
    ):
        params_dict = t_to_dict(params)

        limit = params_dict["limit"]
        page_num = params_dict["pageNum"]
        sw_name = params_dict["swName"]
        level = params_dict["level"]
        search_text = params_dict["searchText"]
        from_date = params_dict["fromDate"]
        to_date = params_dict["toDate"]

        col = db["state_logs"]

        sort_spec = [("createdAt", DESCENDING)]

        query: dict[str, Any] = {}

        if search_text:
            if len(search_text) > 100:
                raise ValueError("searchText must be less than 100 characters")

            query = make_check_search_text_query("contents", search_text, query=query)

        query = make_check_include_query("level", level, query=query)
        query = make_check_include_query("swName", sw_name, query=query)

        if from_date and to_date is None:
            to_date = datetime.now(UTC).isoformat()
        elif from_date is None and to_date:
            raise ValueError("When there is a to_date, there must also be a from_date.")

        if from_date and to_date:
            query = make_check_date_range_query(
                "createdAt", {"from": from_date, "to": to_date}, query=query
            )

        if limit is not None and limit <= 0:
            limit = 30

        total_count = await col.count_documents(query)

        if page_num is not None and page_num > 0:
            if limit is None:
                limit = 30
            skip = (page_num - 1) * limit
        else:
            skip = 0

        cursor = col.find(query).sort(sort_spec)

        if skip:
            cursor = cursor.skip(skip)

        if limit is None:
            docs = [doc async for doc in cursor]
        else:
            cursor = cursor.limit(limit)
            docs = await cursor.to_list(length=limit)

        items = [LogItem.model_validate(d) for d in docs]

        has_next = (skip + len(docs)) < total_count if limit is not None else False

        return {
            "items": items,
            "hasNext": has_next,
            "totalCount": total_count,
        }

    async def error_log_count_for_24h(self, *, db: MongoDB):
        """API 엔드포인트용: DB에서 정확한 값 조회"""
        if self._initialized:
            return {"count": self._error_count}

        await self._ensure_initialized(db)  #자동 초기화

        now_utc = datetime.now(UTC)
        cutoff_dt = now_utc - timedelta(hours=24)

        cnt = await db["state_logs"].count_documents(
            {"level": {"$in": [2, self._level_to_name[2]]}, "createdAtDt": {"$gte": cutoff_dt}}
        )

        # 메모리 카운터도 동기화
        async with self._count_lock:
            self._error_count = cnt

        return {"count": cnt}

    async def warning_log_count_for_24h(self, *, db: MongoDB):
        """API 엔드포인트용: DB에서 정확한 값 조회"""
        if self._initialized:
            return {"count": self._warning_count}

        await self._ensure_initialized(db)  #자동 초기화

        now_utc = datetime.now(UTC)
        cutoff_dt = now_utc - timedelta(hours=24)

        cnt = await db["state_logs"].count_documents(
            {"level": {"$in": [1, self._level_to_name[1]]}, "createdAtDt": {"$gte": cutoff_dt}}
        )

        # 메모리 카운터도 동기화
        async with self._count_lock:
            self._warning_count = cnt

        return {"count": cnt}

    async def on_zenoh_sub_state(self, *, db: MongoDB, topic, obj, attachment):
        rb_log.debug("🔎 subscribe */state_log")
        now_utc = datetime.now(UTC)
        sw_name = topic.split("/")[0]

        obj["swName"] = sw_name
        obj["level"] = (
            obj["level"] if isinstance(obj["level"], int) else (obj["level"] or "unknown").lower()
        )
        obj["createdAt"] = now_utc.isoformat()

        await socket_client.emit("state_log", obj)

        fire_and_log(self.insert_db_and_emit_24h_count(db, obj, now_utc))

    async def on_zenoh_sub_rrs_log(self, *, db: MongoDB, obj):
        rb_log.debug("🔎 subscribe rrs_log")
        now_utc = datetime.now(UTC)
        obj["swName"] = "RRS"
        obj["level"] = (
            obj["level"] if isinstance(obj["level"], int) else (obj["level"] or "unknown").lower()
        )
        obj["createdAt"] = now_utc.isoformat()

        await socket_client.emit("state_log", obj)

        fire_and_log(self.insert_db_and_emit_24h_count(db, obj, now_utc))

    async def insert_db_and_emit_24h_count(self, db: MongoDB, d: dict, now_dt: datetime):
        """로그 삽입 + 실시간 emit (메모리 카운터 사용 - 빠름!)"""
        try:
            level = self._level_to_name.get(d["level"]) or "unknown"
            int_level = int(d["level"])
            d["level"] = level

            d.setdefault("createdAtDt", now_dt)

            last_log = await db["state_logs"].find_one(sort=[("createdAt", -1)])

            if (
                last_log
                and last_log.get("level") == level
                and last_log.get("contents") == d.get("contents")
            ):
                return

            #DB 삽입만 수행 (빠름 - 10ms)
            await db["state_logs"].insert_one(d)

            #메모리 카운터 증가 및 실시간 emit (count 쿼리 없음!)
            async with self._count_lock:
                if int_level == 2 or level == "error":
                    self._error_count += 1
                    await socket_client.emit("error_log_count_for_24h", {"count": self._error_count})
                elif int_level == 1 or level == "warning":
                    self._warning_count += 1
                    await socket_client.emit("warning_log_count_for_24h", {"count": self._warning_count})

        except Exception as e:
            rb_log.error(f"[state_log persist/count error] {e}")

    async def export_state_logs_csv(
        self,
        *,
        db: MongoDB,
        swName: str | None = None,
        level: list[int | str] | int | str | None = None,
        searchText: str | None = None,
        fromDate: str | None = None,
        toDate: str | None = None,
        filename: str | None = None,
    ):

        col = db["state_logs"]

        sort_spec = [("createdAt", DESCENDING)]

        query: dict[str, Any] = {}

        if searchText:
            if len(searchText) > 100:
                raise ValueError("searchText must be less than 100 characters")

            query = make_check_search_text_query("contents", searchText, query=query)

        query = make_check_include_query("level", level, query=query)
        query = make_check_include_query("swName", swName, query=query)

        if fromDate and toDate is None:
            toDate = datetime.now(UTC).isoformat()
        elif fromDate is None and toDate:
            raise ValueError("When there is a toDate, there must also be a fromDate.")

        if fromDate and toDate:
            query = make_check_date_range_query(
                "createdAt", {"from": fromDate, "to": toDate}, query=query
            )

        fields = ["createdAt", "timestamp", "swName", "level", "contents"]

        async def row_generator():
            # Excel(Windows)에서 바로 열리게 BOM (한 번만)
            yield "\ufeff".encode()

            # CSV writer 준비
            sio = io.StringIO()
            writer = csv.writer(sio, lineterminator="\r\n")

            # 헤더 한 번만 출력
            writer.writerow(fields)
            yield sio.getvalue().encode("utf-8")
            sio.seek(0)
            sio.truncate(0)

            cursor = col.find(query).sort(sort_spec)

            async for doc in cursor:
                # <-- 반드시 루프 내부에서 새 리스트 생성
                row = []
                for f in fields:
                    v = doc.get(f)

                    # level 처리: int이면 mapping, 아니면 문자열 처리
                    if f == "level":
                        if isinstance(v, int):
                            v = self._level_to_name.get(v, "Unknown")

                        v = v.capitalize() if v else "Unknown"

                    # createdAt: datetime이면 isoformat, 아니면 str로
                    if isinstance(v, datetime):
                        row.append(v.isoformat())
                    elif isinstance(v, bytes):
                        row.append(v.decode("utf-8", "ignore"))
                    else:
                        row.append(str(v) if v is not None else "")

                writer.writerow(row)
                yield sio.getvalue().encode("utf-8")
                sio.seek(0)
                sio.truncate(0)

        new_filename = sanitize_filename(
            filename or f"state_logs_{datetime.now(UTC).strftime('%Y%m%d_%H%M%S')}.csv"
        )
        headers = {"Content-Disposition": content_disposition_header(new_filename)}

        return StreamingResponse(
            row_generator(),
            media_type="text/csv; charset=utf-8",
            headers=headers,
        )
