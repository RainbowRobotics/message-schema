import csv
import io
from datetime import UTC, datetime, timedelta

from app.features.log.log_schema import (
    LogItem,
    Request_LogListParamsPD,
)
from app.socket.socket_client import socket_client
from fastapi.responses import StreamingResponse
from pymongo import DESCENDING
from rb_database.mongo_db import MongoDB
from rb_database.utils import (
    make_check_date_range_query,
    make_check_include_query,
    make_check_search_text_query,
)
from rb_modules.log import rb_log
from utils.asyncio_helper import fire_and_log
from utils.file import content_disposition_header, sanitize_filename


class LogService:
    def __init__(self):
        self._level_to_name = {
            1: "info",
            2: "warning",
            3: "error",
            4: "user",
            5: "debug",
            6: "general",
        }

    async def get_log_list(
        self,
        *,
        db: MongoDB,
        params: Request_LogListParamsPD,
    ):
        limit = params["limit"]
        pageNum = params["pageNum"]
        swName = params["swName"]
        level = params["level"]
        searchText = params["searchText"]
        fromDate = params["fromDate"]
        toDate = params["toDate"]

        col = db["state_logs"]

        sort_spec = [("createdAt", DESCENDING)]

        query = {}

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

        if limit is not None and limit <= 0:
            limit = 30

        total_count = await col.count_documents(query)

        if pageNum is not None and pageNum > 0:
            if limit is None:
                limit = 30
            skip = (pageNum - 1) * limit
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
        now_utc = datetime.now(UTC)

        cutoff_iso = (now_utc - timedelta(hours=24)).isoformat()

        cnt = await db["state_logs"].count_documents(
            {"level": {"$in": [3, self._level_to_name[3]]}, "createdAt": {"$gte": cutoff_iso}}
        )

        res = {"count": cnt}

        return res

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
        try:
            level = self._level_to_name[d["level"]] or "unknown"
            int_level = int(d["level"])
            d["level"] = level

            last_log = await db["state_logs"].find_one(sort=[("createdAt", -1)])

            if (
                last_log
                and last_log.get("level") == level
                and last_log.get("contents") == d.get("contents")
            ):
                return

            await db["state_logs"].insert_one(d)

            cutoff_iso = (now_dt - timedelta(hours=24)).isoformat()

            cnt = await db["state_logs"].count_documents(
                {"level": {"$in": [int_level, level]}, "createdAt": {"$gte": cutoff_iso}}
            )

            if int_level == 3 or level == "Error":
                await socket_client.emit("error_log_count_for_24h", {"count": cnt})
            elif int_level == 2 or level == "Warning":
                await socket_client.emit("warning_log_count_for_24h", {"count": cnt})

        except Exception as e:
            print(f"[state_log persist/count error] {e}", flush=True)

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

        query = {}

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
