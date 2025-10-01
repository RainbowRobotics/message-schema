import asyncio
import time
from collections.abc import Sequence
from typing import Annotated

from fastapi import Depends, FastAPI, Request
from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorCollection, AsyncIOMotorDatabase
from pymongo.errors import OperationFailure
from rb_modules.log import rb_log

client: AsyncIOMotorClient | None = None
db: AsyncIOMotorDatabase | None = None


async def ensure_index(
    db: AsyncIOMotorDatabase,
    col_name: str,
    keys: Sequence[tuple[str, int]] | tuple[str, int] | str,
    *,
    name: str,
    **create_kwargs,
):
    col = db[col_name]

    if isinstance(keys, str) or (
        isinstance(keys, tuple) and len(keys) == 2 and isinstance(keys[0], str)
    ):
        key_spec = [keys] if isinstance(keys, tuple) else [(keys, 1)]
    else:
        key_spec = list(keys)

    try:
        existing = await col.index_information()
    except Exception as e:
        rb_log.error(f"index_information failed: {e}")
        raise

    if name in existing:
        info = existing[name]

        existing_key = info.get("key")

        normalized_existing_key = [(k, 1 if v == 1 else v) for k, v in existing_key]
        normalized_request_key = [(k, v) for k, v in key_spec]

        same_key = normalized_existing_key == normalized_request_key

        def _opt_val(d, k):
            return d.get(k, None)

        opts_to_check = ["unique", "default_language", "weights", "sparse"]
        same_opts = all(
            _opt_val(info, opt) == create_kwargs.get(opt)
            for opt in opts_to_check
            if opt in info or opt in create_kwargs
        )

        if same_key and same_opts:
            return

        try:
            await col.drop_index(name)
            await asyncio.sleep(0.5)
        except Exception as e:
            rb_log.error(f"[index] drop_index failed: {e}")
            raise

    try:
        created_name = await col.create_index(key_spec, name=name, **create_kwargs)
        rb_log.info(f"[index] created index {created_name} on {col_name}", disable_db=True)
    except OperationFailure as e:
        code = getattr(e, "code", None)
        msg = str(e)
        if code == 85 or "IndexOptionsConflict" in msg:
            print(f"[index] IndexOptionsConflict for {name}: {e}")

            try:
                await col.drop_index(name)
                await asyncio.sleep(0.5)
                created_name = await col.create_index(key_spec, name=name, **create_kwargs)
                rb_log.info(f"[index] recreated index {created_name} after drop", disable_db=True)
            except Exception as e2:
                rb_log.error(f"[index] recreate failed: {e2}")
                raise

        else:
            raise


async def init_indexes(db: AsyncIOMotorDatabase):
    ensure_index(db, "robots", "name", name="robots_name_idx", unique=True)
    ensure_index(
        db,
        "state_logs",
        [("swName", 1), ("level", 1), ("createdAt", -1)],
        name="state_logs_sw_level_created_idx",
    )
    ensure_index(db, "state_logs", [("contents", "text")], name="state_logs_text_idx")


async def init_db(app: FastAPI, uri: str, db_name: str):
    global client, db
    client = AsyncIOMotorClient(uri, uuidRepresentation="standard")
    db = client[db_name]

    await init_indexes(db)

    app.state.mongo_client = client
    app.state.mongo_db = db


async def close_db(app: FastAPI):
    c = app.state.mongo_client

    if c:
        c.close()


def get_db(request: Request) -> AsyncIOMotorDatabase:
    return request.app.state.mongo_db


async def wait_db_ready(timeout: int = 2) -> AsyncIOMotorDatabase:
    start = time.monotonic()
    while db is None:
        print("🔎 wait db ready", flush=True)
        if time.monotonic() - start > timeout:
            raise RuntimeError("MongoDB not ready")
        await asyncio.sleep(0.05)


MongoDB = Annotated[AsyncIOMotorDatabase, Depends(get_db)]
MongoCollection = AsyncIOMotorCollection
