"""
mongo_db.py
"""
import asyncio
import gzip
import os
import time
from collections.abc import (
    Sequence,
)
from datetime import (
    UTC,
    date,
    datetime,
    timedelta,
)
from typing import (
    Annotated,
    Any,
)
from zoneinfo import ZoneInfo

from bson import json_util
from fastapi import (
    Depends,
    FastAPI,
)
from motor.motor_asyncio import (
    AsyncIOMotorClient,
    AsyncIOMotorCollection,
    AsyncIOMotorDatabase,
)
from pymongo import (
    MongoClient,
)
from pymongo.errors import (
    OperationFailure,
    PyMongoError,
)
from rb_modules.log import (
    rb_log,
)
from rb_utils.date import (
    get_current_dt_yyyymmddhhmmss,
)
from rb_utils.parser import (
    _normalize_filter,
)

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
        except Exception:   # pylint: disable=broad-exception-caught
            pass
        print(f"⏳ Waiting for PRIMARY election... {i+1}s", flush=True)
        await asyncio.sleep(1)

    raise RuntimeError("Primary not elected within timeout")

async def ensure_index(
    db_: AsyncIOMotorDatabase,
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
    col = db_[col_name]
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
                except Exception as e2:  # pylint: disable=broad-exception-caught
                    rb_log.error(f"[index] conflict resolution failed on {col_name}.{name}: {e2}")
                    # 경합 가능성이 있으니 한 번 더 재시도 여지 제공
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, 1.0)
                    continue

            # 그 외 에러는 즉시 전파
            raise
        except Exception as e:  # pylint: disable=broad-exception-caught
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
    """
    [인덱스 초기화]
    - 비동기 함수로 처리
    - 데이터베이스 인덱스 초기화
    """
    await ensure_index(db, "robots", "name", name="robots_name_idx", unique=True)
    await ensure_index(
        db,
        "state_logs",
        [("swName", 1), ("level", 1), ("createdAt", -1)],
        name="state_logs_sw_level_created_idx",
    )
    await ensure_index(db, "state_logs", [("contents", "text")], name="state_logs_text_idx")
    await ensure_index(
        db,
        "state_logs",
        [("createdAtDt", 1)],
        name="state_logs_createdAtDt_ttl",
        expireAfterSeconds=60 * 60 * 24 * 90,  # 90일
    )
    await ensure_index(db, "programs", [("name", 1)], name="uniq_program_name", unique=True)


async def init_db(app: FastAPI, uri: str, db_name: str):
    """
    [DB 초기화]
    - 비동기 함수로 처리
    - 데이터베이스 초기화
    """
    global client, db, py_mongo_client
    client = AsyncIOMotorClient(uri, uuidRepresentation="standard")
    py_mongo_client = MongoClient(uri)
    db = client[db_name]

    await ensure_replica_set(client)
    await init_indexes(db)

    app.state.mongo_client = client
    app.state.mongo_db = db

async def close_db(app: FastAPI):
    """
    [DB 종료]
    - 비동기 함수로 처리
    - 데이터베이스 종료
    """
    c = app.state.mongo_client

    if c:
        c.close()


async def wait_db_ready(timeout: int = 15):
    """
    [DB 준비 대기]
    - 비동기 함수로 처리
    - 데이터베이스 준비 대기
    - 데이터베이스가 준비되지 않으면 예외 발생
    - timeout: 최대 대기 시간 (기본 15초)
    """
    start = time.monotonic()
    while db is None:
        print("🔎 wait db ready", flush=True)
        if time.monotonic() - start > timeout:
            raise RuntimeError("MongoDB not ready")
        await asyncio.sleep(0.05)

async def get_db():
    """
    [DB 조회]
    - 비동기 함수로 처리
    - 데이터베이스 준비 대기
    - 데이터베이스가 준비되지 않으면 예외 발생
    """
    await wait_db_ready()

    if db is None:
        raise RuntimeError("Database not initialized after waiting")

    return db



# === Yujin Added ===

async def add_collection(_db: AsyncIOMotorDatabase, col_name: str) -> bool:
    """
    [컬렉션 생성]
    - 이미 존재하면 패스
    """
    if col_name is None:
        return False

    await wait_db_ready()

    names = await _db.list_collection_names()
    if col_name not in names:
        await _db.create_collection(col_name)
        return True

    return False

async def get_collection_stats(_db: AsyncIOMotorDatabase, col_name: str, scale: int = 1):
    """
    [컬렉션 상태 조회]
    - 컬렉션 이름이 없으면 None 반환
    - scale: 바이트->KB, MB 등 단위 스케일 (기본 byte, KB 단위로 변환하려면 scale=1024)
    """
    if col_name is None:
        return None

    await wait_db_ready()

    return await _db.command({"collStats": col_name, "scale": scale})

async def archive_collection(
    col: AsyncIOMotorCollection,
    cutoff_utc: date | datetime,
    out_dir: str,
    dry_run: bool = False,
    tz_name: str = "Asia/Seoul"
    ) -> list[dict]:
    """
    [아카이브 + db삭제]
    - 일정 날짜 기준으로 이전은 전부 날짜별로 압축보관 + db에서 삭제
    - 내부적으로만 사용
    - DB 내 저장된 데이터의 timezone은 UTC이지만 저장되는 데이터는 KST 기준으로 자름
        - cutoff_utc : 아카이브 할 날짜 기준 UTC(해당날짜 이전의 로그를 전부 아카이브 + 삭제)
        - out_dir : 저장디렉토리. /data/{server}/archive/{service}로 통일
        - filters : 검색 필터.
    """
    pipeline = [
        {"$match": {"createdAt": {"$lt": cutoff_utc}}},
        {"$project": {
            "_id": 0,
            "day": {
                "$dateToString": {
                    "date": "$createdAt",
                    "timezone": tz_name,
                    "format": "%Y-%m-%d"
                }
            }
        }},
        {"$group": {"_id": "$day", "count": {"$sum": 1}}},
        {"$sort": {"_id": 1}},
    ]

    print(f"[archive_collection] pipeline: {pipeline}")

    # 3) 데이터 조회
    day_rows = await col.aggregate(pipeline, allowDiskUse=True).to_list(length=None)
    if not day_rows:
        return []

    os.makedirs(out_dir, exist_ok=True)
    results: list[dict] = []

    hint_opt = None
    try:
        idxinfo = await col.index_information()
        if "createdAt_1" in idxinfo:
            hint_opt = "createdAt_1"
    except PyMongoError as e:
        rb_log.error(f"[archive_collection] index_information failed: {e}")

    for row in day_rows:
        day = row["_id"]        # 'YYYY-MM-DD'
        count_est = int(row.get("count", 0))

        y, m, d = map(int, day.split("-"))
        start_kst = datetime(y, m, d, 0, 0, 0, tzinfo=ZoneInfo(tz_name))
        end_kst   = start_kst + timedelta(days=1)
        start_utc = start_kst.astimezone(UTC)
        end_utc   = end_kst.astimezone(UTC)

        fname = f"{day}.ndjson.gz"
        tmp_path = os.path.join(out_dir, fname + ".tmp")
        final_path = os.path.join(out_dir, fname)

        archived_count = 0
        gz_bytes = 0
        deleted_count = 0

        day_filter = {"createdAt": {"$gte": start_utc, "$lt": end_utc}}

        if not dry_run:
            try:
                cursor = col.find(
                    day_filter,
                    projection=None,
                    batch_size=10_000,
                    sort=[("_id", 1)],
                    hint=hint_opt
                )

                print("path : ", tmp_path, final_path)
                with gzip.open(tmp_path, mode="wt", encoding="utf-8") as gz:
                    async for doc in cursor:
                        gz.write(json_util.dumps(doc))
                        gz.write("\n")
                        archived_count += 1

                gz_bytes = os.path.getsize(tmp_path)
                os.replace(tmp_path, final_path)

                if archived_count > 0:
                    delete_kwargs = {}
                    if hint_opt:
                        delete_kwargs["hint"] = hint_opt
                    delete_result = await col.delete_many(day_filter, **delete_kwargs)
                    deleted_count = delete_result.deleted_count

            except PyMongoError as e:
                try:
                    if os.path.exists(tmp_path):
                        os.remove(tmp_path)
                except OSError:
                    pass
                results.append({
                    "day": day,
                    "estimatedDocs": count_est,
                    "archivedDocs": archived_count,
                    "deletedDocs": deleted_count,
                    "file": None,
                    "size": None,
                    "error": repr(e)
                })
                continue

        results.append({
            "day": day,
            "estimatedDocs": count_est,
            "archivedDocs": archived_count if not dry_run else 0,
            "deletedDocs": deleted_count if not dry_run else 0,
            "file": None if dry_run else (final_path if archived_count > 0 else None),
            "size": None if dry_run else (gz_bytes if archived_count > 0 else None),
        })

    return results

async def export_collection(
    col: AsyncIOMotorCollection,
    start_utc: datetime | None = None,
    end_utc: datetime | None = None,
    out_dir: str | None = None,
    file_name: str | None = None,
    filters: str | dict[str, Any] | None = None,
    fields: dict[str, Any] | None = None,
    search_text: str | None = None,
    sort: list[tuple[str, int]] | None = None,
    order: str | None = None,
) -> dict:
    """
    [필터 아카이브]
    - 외부적으로도 사용
    - 필터를 기반으로 DB에서 뽑아낸 로그들을 파일로 압축 저장 -> 이후 데이터전송 등으로 사용
    - DB 내 저장된 데이터의 timezone은 UTC이지만 저장되는 데이터는 KST 기준으로 자름
        - out_dir : 저장디렉토리. /data/{server}/archive/{service}로 통일
        - file_name : 파일 이름. 입력 시 사용
        - filters : 검색 필터. 각 컬럼의 검색어라던가 날짜시간 등
    """
    print("========================================")
    # 1) 필터 파싱
    filters = _normalize_filter(filters)
    tz_name = "Asia/Seoul"

    # 2) 조회 조건 구성
    created_cond: dict[str, Any] = {}
    if start_utc is not None:
        created_cond["$gte"] = start_utc
    if end_utc is not None:
        created_cond["$lt"] = end_utc

    # 3) 필드 세팅(_id 제외하는 것 일괄로 추가)
    if fields is None:
        fields = {"_id": 0}
    else:
        fields = {**fields, "_id": 0}

    # 4) 텍스트 검색(q) -> checkDB에서 등록한 인덱스안에서 검색
    if search_text:
        filters["$text"] = {"$search": search_text}

    # 5) 정렬: 안정 정렬을 위해 보조키로 _id도 포함
    sort_spec = [(sort, order)]
    if sort != "_id":
        sort_spec.append(("_id", order))

    if created_cond:
        filters["createdAt"] = created_cond

    # 3) 경로 생성
    os.makedirs(out_dir, exist_ok=True)

    # 4) 파일 이름 결정
    if not file_name:
        file_name = get_current_dt_yyyymmddhhmmss(tz_name)
    if not file_name.endswith(".ndjson.gz"):
        file_name = f"{file_name}.ndjson.gz"

    tmp_path = os.path.join(out_dir, f"{file_name}.tmp")
    final_path = os.path.join(out_dir, file_name)

    # 5) 데이터 예상 조회
    # results: list[dict] = []
    archived_count = 0
    gz_bytes = 0


    print("filters: ", filters)
    count_est = await col.count_documents(filters)
    print(f"[export_collection] count_est: {count_est}")
    if count_est == 0:
        return {
            "estimatedDocs": 0,
            "archivedDocs": 0,
            "file": None,
            "size": None,
        }

    # 6) 데이터 조회 및 반환
    try:
        cursor = col.find(filters,fields).sort(sort_spec)
        with gzip.open(tmp_path, mode="wt", encoding="utf-8") as gz:
            async for doc in cursor:
                gz.write(json_util.dumps(doc))
                gz.write("\n")
                archived_count += 1
        print(f"[export_collection] archived_count: {archived_count}")
        gz_bytes = os.path.getsize(tmp_path)
        os.replace(tmp_path, final_path)

        return {
            "estimatedDocs": count_est,
            "archivedDocs": archived_count,
            "file": final_path if archived_count > 0 else None,
            "size": gz_bytes if archived_count > 0 else None,
            "error": None,
        }

    except PyMongoError as e:
        try:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        except OSError:
            pass
        print("ERRORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
        print(f"[export_collection] error: {e}")
        return {
            "estimatedDocs": count_est,
            "archivedDocs": archived_count,
            "file": None,
            "size": None,
            "error": repr(e),
        }




MongoDB = Annotated[AsyncIOMotorDatabase, Depends(get_db)]
MongoCollection = AsyncIOMotorCollection
