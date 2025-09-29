from datetime import UTC, datetime, timedelta

from app.features.log.log_schema import LogItem
from app.socket.socket_client import socket_client
from rb_database.mongo_db import MongoDB
from rb_modules.log import rb_log
from utils.asyncio_helper import fire_and_log


class LogService:
    def __init__(self):
        self._level_to_name = {
            1: "Info",
            2: "Warning",
            3: "Error",
            4: "User",
            5: "Debug",
            6: "General",
        }

    async def get_log_list(self, *, db: MongoDB, limit: int = 30):
        cursor = db["state_logs"].find({}).sort([("createdAt", -1), ("_id", -1)]).limit(limit=limit)
        docs = await cursor.to_list(length=limit)

        items = [LogItem.model_validate(d) for d in docs]

        res = {"items": items}
        return res

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
        obj["level"] = int(obj["level"] or 0)
        obj["createdAt"] = now_utc.isoformat()

        await socket_client.emit("state_log", obj)

        fire_and_log(self.persist_and_emit_24h_count(db, obj, now_utc))

    async def on_zenoh_sub_rrs_log(self, *, db: MongoDB, obj):
        rb_log.debug("🔎 subscribe rrs_log")
        now_utc = datetime.now(UTC)
        obj["swName"] = "RRS"
        obj["level"] = int(obj["level"] or 0)
        obj["createdAt"] = now_utc.isoformat()

        await socket_client.emit("state_log", obj)

        fire_and_log(self.persist_and_emit_24h_count(db, obj, now_utc))

    async def persist_and_emit_24h_count(self, db: MongoDB, d: dict, now_dt: datetime):
        try:
            level = self._level_to_name[d["level"]] or "Unknown"
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
