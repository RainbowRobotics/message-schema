import asyncio
from datetime import UTC, datetime, timedelta

import rb_database.mongo_db as mongo_db
from app.socket.socket_client import socket_client
from fastapi import HTTPException
from flat_buffers.IPC.State_Log import State_LogT
from rb_zenoh.router import ZenohRouter

zenoh_log_router = ZenohRouter()


@zenoh_log_router.subscribe("*/state_log", flatbuffer_obj_t=State_LogT)
async def on_zenoh_sub_state(*, topic, mv, obj, attachment):
    try:
        print("🔎 subscribe */state_log", flush=True)
        now_utc = datetime.now(UTC)
        sw_name = topic.split("/")[0]

        obj["swName"] = sw_name
        obj["level"] = int(obj["level"] or 0)
        obj["createdAt"] = now_utc.isoformat()

        await socket_client.emit("state_log", obj)

        level = obj["level"]

        async def persist_and_emit_24h_count(d: dict, lv: int, now_dt: datetime):
            try:
                await mongo_db.db["state_logs"].insert_one(d)

                cutoff_iso = (now_dt - timedelta(hours=24)).isoformat()

                cnt = await mongo_db.db["state_logs"].count_documents(
                    {"level": lv, "createdAt": {"$gte": cutoff_iso}}
                )

                if lv == 2:
                    await socket_client.emit("error_log_count_for_24h", {"count": cnt})
                elif lv == 1:
                    await socket_client.emit("warning_log_count_for_24h", {"count": cnt})

            except Exception as e:
                print(f"[state_log persist/count error] {e}", flush=True)

        asyncio.create_task(persist_and_emit_24h_count(obj, level, now_utc))

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(f"error: {e}")) from e
