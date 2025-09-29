import asyncio
import time
from typing import Annotated

from fastapi import Depends, FastAPI, Request
from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorCollection, AsyncIOMotorDatabase

client: AsyncIOMotorClient | None = None
db: AsyncIOMotorDatabase | None = None


async def init_indexes(db: AsyncIOMotorDatabase):
    await db["robots"].create_index("name", unique=True)
    await db["state_logs"].create_index([("swName", 1), ("createdAt", -1)])


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
