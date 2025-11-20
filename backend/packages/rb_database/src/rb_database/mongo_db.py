import asyncio
import time
from collections.abc import Sequence
from typing import Annotated, Any

from fastapi import Depends, FastAPI
from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorCollection, AsyncIOMotorDatabase
from pymongo import MongoClient
from pymongo.errors import OperationFailure
from rb_modules.log import rb_log

client: AsyncIOMotorClient | None = None
py_mongo_client: MongoClient | None = None
db: AsyncIOMotorDatabase | None = None


def _norm_key_spec(keys: Sequence[tuple[str, int | str]] | tuple[str, int | str] | str):
    if isinstance(keys, str) or (
        isinstance(keys, tuple) and len(keys) == 2 and isinstance(keys[0], str)
    ):
        key_spec = [keys] if isinstance(keys, tuple) else [(keys, 1)]
    else:
        key_spec = list(keys)
    # 1은 1로, 그 외(예: "text", -1)는 그대로
    return [(k, 1 if v == 1 else v) for k, v in key_spec], key_spec


async def _find_index_name_by_keys(col, norm_req):
    existing = await col.index_information()
    for idx_name, meta in existing.items():
        ex_key = meta.get("key") or []
        norm_ex = [(k, 1 if v == 1 else v) for k, v in ex_key]
        if norm_ex == norm_req:
            return idx_name, meta
    return None, None


async def ensure_replica_set(
    motor_client: AsyncIOMotorClient,
    *,
    rs_name: str = "rs0",
    seed_host: str = "rrs-mongo-dev:27017",
):
    """
    1) rs.status로 활성 여부 확인 (Motor)
    2) NotYetInitialized면 동기 pymongo로 rs.initiate() 1회 수행
    3) PRIMARY 선출될 때까지 hello로 대기 (Motor)
    """
    # 1) 이미 활성화?
    try:
        await motor_client.admin.command("replSetGetStatus")
        print("✅ Replica set already active.")
        return
    except OperationFailure as e:
        msg = str(e)
        if "not running with --replSet" in msg:
            print("❌ mongod가 --replSet 없이 실행됨. 컨테이너 command 확인 필요.")
            return
        if "no replset config has been received" in msg:
            # 2) initiate (동기 pymongo 사용: await 금지)
            print("🔧 Initializing replica set...")
            sync_client: MongoClient = MongoClient(
                f"mongodb://{seed_host}"
            )  # RS 파라미터 없이 단일 연결
            try:
                sync_client.admin.command(
                    "replSetInitiate",
                    {"_id": rs_name, "members": [{"_id": 0, "host": seed_host}]},
                )
                print("✅ rs.initiate() sent")
            except OperationFailure as e2:
                # 이미 초기화된 경우 포함
                if "already initialized" in str(e2):
                    print("ℹ️ Replica set already initialized (server says).")
                else:
                    raise
            finally:
                sync_client.close()
        else:
            # 다른 오류는 그대로 위로
            raise

    # 3) PRIMARY 선출 대기 (hello: MongoDB 5+)
    for i in range(90):  # 최대 90초
        try:
            hello = await motor_client.admin.command({"hello": 1})
            if hello.get("isWritablePrimary"):
                print(f"🎉 PRIMARY ready: {hello.get('primary') or seed_host}")
                return
        except Exception:
            pass
        print(f"⏳ Waiting for PRIMARY election... {i+1}s", flush=True)
        await asyncio.sleep(1)

    raise RuntimeError("Primary not elected within timeout")


async def ensure_index(
    db: AsyncIOMotorDatabase,
    col_name: str,
    keys: Sequence[tuple[str, int | str]] | tuple[str, int | str] | str,
    *,
    name: str,
    max_retries: int = 5,
    **create_kwargs,
):
    """
    동시성 안전:
    1) 먼저 create_index 시도
    2) 옵션 충돌/이름 충돌일 때만 정확히 타깃을 찾아 드롭 후 재생성
    3) IndexBuildAborted(276)는 짧게 대기하고 재시도(백오프)
    """
    col = db[col_name]
    norm_req, raw_spec = _norm_key_spec(keys)

    # 이미 동일 키의 인덱스가 있고 옵션도 동일하면 skip
    try:
        existing = await col.index_information()
    except Exception as e:
        rb_log.error(f"[index] index_information failed: {e}")
        raise

    matched_name = None
    matched_meta: dict[str, Any] = {}
    for idx_name, meta in existing.items():
        ex_key = meta.get("key") or []
        if [(k, 1 if v == 1 else v) for k, v in ex_key] == norm_req:
            matched_name, matched_meta = idx_name, meta
            break

    if matched_name:
        opts_to_check = [
            "unique",
            "sparse",
            "default_language",
            "weights",
            "expireAfterSeconds",
            "collation",
        ]
        same_opts = all(
            (matched_meta.get(o) == create_kwargs.get(o))
            for o in opts_to_check
            if (o in matched_meta) or (o in create_kwargs)
        )
        if same_opts:
            return

    # 재시도 루프
    delay = 0.1
    for attempt in range(1, max_retries + 1):
        try:
            created_name = await col.create_index(raw_spec, name=name, **create_kwargs)
            return
        except OperationFailure as e:
            code = getattr(e, "code", None)
            msg = str(e)

            # 1) 다른 인스턴스가 dropIndexes를 쏴서 빌드 중단됨 → 재시도
            if code == 276 or "IndexBuildAborted" in msg or "dropIndexes command" in msg:
                rb_log.warning(
                    f"[index] build aborted by concurrent drop/create on {col_name}.{name}; "
                    f"retrying attempt={attempt}/{max_retries}"
                )
                await asyncio.sleep(delay)
                delay = min(delay * 2, 1.0)
                continue

            # 2) 옵션 충돌 / 이름만 다른 동일키 충돌 → 타깃 찾아 정확히 드롭 후 재생성
            if code == 85 or "IndexOptionsConflict" in msg or "different name" in msg:
                try:
                    # 현재 존재하는 동일 키/다른 이름 인덱스 찾기
                    target_name, _ = await _find_index_name_by_keys(col, norm_req)
                    if not target_name:
                        existing = await col.index_information()
                        if name in existing:
                            target_name = name

                    if target_name:
                        await asyncio.sleep(0.05)
                        await col.drop_index(target_name)
                        await asyncio.sleep(0.05)

                    created_name = await col.create_index(raw_spec, name=name, **create_kwargs)
                    rb_log.info(
                        f"[index] recreated {col_name}.{created_name} after resolving conflict",
                        disable_db=True,
                    )
                    return
                except Exception as e2:
                    rb_log.error(f"[index] conflict resolution failed on {col_name}.{name}: {e2}")
                    # 경합 가능성이 있으니 한 번 더 재시도 여지 제공
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, 1.0)
                    continue

            # 그 외 에러는 즉시 전파
            raise
        except Exception as e:
            # 비정형 에러도 짧게 재시도 (네트워크 등 일시 이슈)
            rb_log.warning(
                f"[index] unexpected error on {col_name}.{name}: {e}; retry {attempt}/{max_retries}",
                disable_db=True,
            )
            await asyncio.sleep(delay)
            delay = min(delay * 2, 1.0)
            continue

    raise RuntimeError(f"ensure_index failed on {col_name}.{name} after {max_retries} attempts")


async def init_indexes(db: AsyncIOMotorDatabase):
    await ensure_index(db, "robots", "name", name="robots_name_idx", unique=True)
    await ensure_index(
        db,
        "state_logs",
        [("swName", 1), ("level", 1), ("createdAt", -1)],
        name="state_logs_sw_level_created_idx",
    )
    await ensure_index(db, "state_logs", [("contents", "text")], name="state_logs_text_idx")
    await ensure_index(db, "programs", [("name", 1)], name="uniq_program_name", unique=True)


async def init_db(app: FastAPI, uri: str, db_name: str):
    global client, db, py_mongo_client
    client = AsyncIOMotorClient(uri, uuidRepresentation="standard")
    py_mongo_client = MongoClient(uri)
    db = client[db_name]

    await ensure_replica_set(client)
    await init_indexes(db)

    app.state.mongo_client = client
    app.state.mongo_db = db


async def close_db(app: FastAPI):
    c = app.state.mongo_client

    if c:
        c.close()


async def wait_db_ready(timeout: int = 15):
    start = time.monotonic()
    while db is None:
        print("🔎 wait db ready", flush=True)
        if time.monotonic() - start > timeout:
            raise RuntimeError("MongoDB not ready")
        await asyncio.sleep(0.05)


async def get_db():
    await wait_db_ready()

    if db is None:
        raise RuntimeError("Database not initialized after waiting")

    return db


MongoDB = Annotated[AsyncIOMotorDatabase, Depends(get_db)]
MongoCollection = AsyncIOMotorCollection
