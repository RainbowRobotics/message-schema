"""
[Move MongoDB 어댑터]
"""
import math
from datetime import (
    UTC,
    date,
    datetime,
)
from typing import (
    Any,
)

import rb_database.mongo_db as mongo_db
from motor.motor_asyncio import (
    AsyncIOMotorCollection,
)
from pymongo import (
    ASCENDING,
    DESCENDING,
)
from rb_modules.log import (
    rb_log,
)
from rb_utils.date import (
    convert_dt,
)
from rb_utils.pagination import (
    LogsResponse,
)
from rb_utils.service_exception import (
    ServiceException,
)

from app.features.move.port.database_port import DatabasePort


class MoveMongoDatabaseAdapter(DatabasePort):
    """
    [Move MongoDB 어댑터 초기화]
    """
    
    def __init__(self):
        self.name = "command_move"

    async def check_db(self):
        try:
            # 0) 인덱스 삭제용..(필요할때만..주석해제..)
            # await mongo_db.db[self.name].drop_index("move_index")
            # await mongo_db.db[self.name].drop_index("created_at_1")

            # 1) 컬렉션 생성(이미 존재하면 패스)
            await mongo_db.add_collection(mongo_db.db, self.name)

            # 2) 인덱스 생성(이미 존재하면 패스)
            await mongo_db.ensure_index(mongo_db.db, self.name, [
                ("message", "text"),("goalPose", "text"),
                ("result", "text"),("status", "text"),
                ("method", "text"),("id", "text"),
                ("goalId", "text"),("command", "text"),
                ("mapName", "text"),
                ("direction", "text"),("createdAt", -1)], name="move_index")

            await mongo_db.ensure_index(mongo_db.db, self.name, [("createdAt", 1)], name="created_at_1")

            # 3) 현재 컬렉션 사이즈 조회(디버그용)
            stats = await mongo_db.get_collection_stats(mongo_db.db, self.name, 1024)
            rb_log.info(f"[stats] {self.name}: {stats['count']} docs, "
                        f"dataSize={stats['size']}KB, storageSize={stats['storageSize']}KB, "
                        f"indexSize={stats['totalIndexSize']}KB")
        except ServiceException as e:
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] checkDB Error : {e}")
            raise ServiceException("DB 초기화 실패", status_code=500) from e

    async def save(self, command: dict):
        try:
            if command is None:
                raise ServiceException("command값이 없습니다", status_code=400)
            await self.check_db()
            await mongo_db.db[self.name].insert_one(command)
        except ServiceException as e:
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] Save Error : {e}")
            raise ServiceException("DB 저장 실패", status_code=500) from e

    async def upsert(self, command: dict):
        try:
            if command is None:
                raise ServiceException("command값이 없습니다", status_code=400)
            await self.check_db()
            command["updateAt"] = datetime.now(UTC)
            await mongo_db.db[self.name].update_one({"id": command["id"]}, {"$set": command}, upsert=True)
        except ServiceException as e:
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] Upsert Error : {e}")
            raise ServiceException("DB 업데이트 실패", status_code=500) from e

    async def update(self, command: dict):
        try:
            if command is None:
                raise ServiceException("command값이 없습니다", status_code=400)
            await self.check_db()
            await mongo_db.db[self.name].update_one({"id": command["id"]}, {"$set": command})
        except ServiceException as e:
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] Update Error : {e}")
            raise ServiceException("DB 업데이트 실패", status_code=500) from e

    async def get_log_by_id(self, id_: str) -> dict:
        try:
            if id_ is None:
                raise ServiceException("id_값이 없습니다", status_code=400)
            return await mongo_db.db[self.name].find_one({"id": id_})
        except ServiceException as e:
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] getLogById Error : {e}")
            raise ServiceException("DB 조회 실패", status_code=500) from e

    async def get_logs(self, options: dict) -> LogsResponse:
        """
        [Move DB 조회 및 페이지네이션]
        - options:
            - filters: dict              # Mongo 조건식
            - searchText: str           # $text 검색어 (텍스트 인덱스 필요)
            - fields: dict              # projection 예) {"_id":0, "contents":1}
            - sort: str                 # 정렬 필드 (기본: createdAt)
            - order: str                # "desc"(기본) | "asc"
            - page: int                 # 기본 1
            - limit: int                # 기본 20, 최대 200
        """
        try:
            # 1) DB 세팅
            await self.check_db()
            col = mongo_db.db[self.name]

            rb_log.info(f"[AmrMove] getLogs : {options}")

            # 2) 옵션 파싱
            filter_options = options.get("filter") or {}
            search_text = options.get("searchText")
            fields = dict(options.get("fields"))
            sort   = options.get("sort", "createdAt")
            order  = DESCENDING if str(options.get("order", "desc")).lower() == "desc" else ASCENDING
            page   = max(1, int(options.get("page", 1)))
            limit  = max(1, min(int(options.get("limit", 20)), 200))
            skip   = (page - 1) * limit

            print(f"[getLogs] filter_options: {filter_options}, search_text: {search_text}, fields: {fields}, sort: {sort}, order: {order}, page: {page}, limit: {limit}, skip: {skip}")

            # 3) 필드 세팅(_id 제외하는 것 일괄로 추가)
            fields = {**fields, "_id": 0}

            # 4) 텍스트 검색(q) -> checkDB에서 등록한 인덱스안에서 검색
            if search_text:
                filter_options["$text"] = {"$search": search_text}

            # 5) 정렬: 안정 정렬을 위해 보조키로 _id도 포함
            sort_spec = [(sort, order)]
            if sort != "_id":
                sort_spec.append(("_id", order))

            # 6) 조회 및 페이지네이션
            cursor = col.find(filter_options, fields).sort(sort_spec).skip(skip).limit(limit)
            items  = await cursor.to_list(length=limit)
            total  = await col.count_documents(filter_options)
            pages  = max(1, math.ceil(total / limit)) if total else 1

            # 7) 응답 반환
            return {
                "items": items,
                "pageInfo": {
                    "mode": "offset",
                    "page": page,
                    "pages": pages,
                    "limit": limit,
                    "total": total,
                    "sort": sort,
                    "order": "desc" if order == DESCENDING else "asc",
                },
            }
        except ServiceException as e:
            raise e

    async def archive_logs(self,
    cutoff_utc: datetime | date,
    dry_run: bool
    ) -> dict:
        """
        [Move DB 로그 아카이브]
        - cutoff_utc: datetime | date # 아카이브 기준 날짜(해당날짜기준부터 이전날짜를 모두 아카이브합니다)
        - dry_run: bool               # 값이 True이면 실제 기능을 수행하진 않고 예상 결과를 반환합니다
        """
        try:
            tz_name = "Asia/Seoul"
            out_dir = "/app/data/amr/archive/move"
            rb_log.info(f"[AmrMove] archiveLogs : cutoff_utc: {cutoff_utc}, tz_name: {tz_name}, out_dir: {out_dir}, dry_run: {dry_run}")

            # 1) 날짜 변환 (KST -> UTC)
            cutoff_utc = convert_dt(local=cutoff_utc, in_tz=tz_name, out_tz="UTC")
            rb_log.info(f"[AmrMove] archiveLogs : {cutoff_utc}")

            # 2) 컬렉션 세팅
            col: AsyncIOMotorCollection = mongo_db.db[self.name]

            # 3) createdAt 문자열 변환 (안전장치 ; createdAt가 문자열인 경우 변환)
            await self.convert_created_at_strings(col, tz_name)

            result = await mongo_db.archive_collection(col, cutoff_utc, out_dir=out_dir, dry_run=dry_run, tz_name=tz_name)
            print(f"[archiveLogs] result: {result}")
            return result
        except ServiceException as e:
            print(f"[archiveLogs] ServiceException: {e}")
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] Delete Logs Error : {e}")
            raise ServiceException("DB 로그 삭제 실패", status_code=500) from e

    async def convert_created_at_strings(self, col: AsyncIOMotorCollection, kst: str = "Asia/Seoul"):
        """
        [createdAt 문자열 변환 (안전장치 ; createdAt가 문자열인 경우 변환)]
        - col: AsyncIOMotorCollection # 컬렉션
        - kst: str                    # 시간대
        """
        res = await col.update_many(
            {"createdAt": {"$type": "string"}},
            [
                {"$set": {
                    "createdAt": {
                        "$let": {
                            "vars": {"s": "$createdAt"},
                            "in": {
                                "$switch": {
                                    "branches": [
                                        # 2025-11-07T01:15:08.577Z
                                        {
                                            "case": {"$regexMatch": {"input": "$$s", "regex": r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$"}},
                                            "then": {"$dateFromString": {"dateString": "$$s"}}
                                        },
                                        # 2025-11-07T01:15:08Z
                                        {
                                            "case": {"$regexMatch": {"input": "$$s", "regex": r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$"}},
                                            "then": {"$dateFromString": {"dateString": "$$s"}}
                                        },
                                        # 2025-11-07 10:15:08  (오프셋 없음 → KST로 간주)
                                        {
                                            "case": {"$regexMatch": {"input": "$$s", "regex": r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$"}},
                                            "then": {"$dateFromString": {
                                                "dateString": "$$s",
                                                "format": "%Y-%m-%d %H:%M:%S",
                                                "timezone": kst
                                            }}
                                        },
                                    ],
                                    # 기본: ISO8601 대부분 커버. 실패 시 null
                                    "default": {"$dateFromString": {"dateString": "$$s", "onError": None, "onNull": None}}
                                }
                            }
                        }
                    }
                }}
            ]
        )
        print(f"[migrate] createdAt strings→Date matched={res.matched_count} modified={res.modified_count}")

    async def export_logs(self, start_dt: datetime | date, end_dt: datetime | date, filters: dict[str, Any], filename: str, search_text: str, fields: dict[str, Any], sort: str, order: str) -> dict:
        """
        [Move DB 로그 내보내기]
        - start_dt: datetime | date # 내보내기 기준 시작 날짜(해당날짜기준부터 내보냅니다)
        - end_dt: datetime | date   # 내보내기 기준 종료 날짜(해당날짜기준까지 내보냅니다)
        - filters: dict            # Mongo 조건식
        - filename: str             # 내보내기 파일명(확장자는 gz로 고정됩니다)
        - search_text: str           # $text 검색어 (텍스트 인덱스 필요)
        - fields: dict              # projection 예) {"_id":0, "contents":1}
        - sort: str                 # 정렬 필드 (기본: createdAt)
        - order: str                # "desc"(기본) | "asc"
        """
        try:
            # 1) DB 세팅
            await self.check_db()
            col = mongo_db.db[self.name]
            tz_name = "Asia/Seoul"
            out_dir = "/app/data/amr/export/move"

            rb_log.info(f"[exportLogs] exportLogs : {start_dt}, {end_dt}, {filters}, {filename}, {search_text}, {fields}, {sort}, {order}")

            # 2) 옵션 파싱
            start_utc = convert_dt(local=start_dt, in_tz=tz_name, out_tz="UTC")
            end_utc = convert_dt(local=end_dt, in_tz=tz_name, out_tz="UTC")
            # filter = filters or {}
            # fields = dict(fields)
            order  = DESCENDING if str(order).lower() == "desc" else ASCENDING

            rb_log.info(f"[exportLogs] start_utc: {start_utc}, end_utc: {end_utc}, filename: {filename}, filters: {filters}, search_text: {search_text}, fields: {fields}, sort: {sort}, order: {order}")

            # 3) 내보내기 실행
            res:dict = await mongo_db.export_collection(
                col=col,
                start_utc=start_utc,
                end_utc=end_utc,
                out_dir=out_dir,
                file_name=filename,
                filters=filters,
                search_text=search_text,
                fields=fields,
                sort=sort,
                order=order
            )

            rb_log.info("---------------------------------")
            rb_log.info(f"[exportLogs] results: {res}")

            return {
                "estimatedDocs": res.get("estimatedDocs"),
                "archivedDocs": res.get("archivedDocs"),
                "file": res.get("file") if res.get("file") else None,
                "size": res.get("size") if res.get("size") else None,
                "error": res.get("error") if res.get("error") else None,
                "meta": {
                    "start_utc": start_utc,
                    "end_utc": end_utc,
                    "filters": filters,
                    "searchText": search_text,
                    "filename": filename,
                    "fields": fields,
                    "sort": sort,
                    "order": order,
                }
            }
        except ServiceException as e:
            rb_log.error(f"[exportLogs] ServiceException: {e}")
            raise e
        except Exception as e:
            rb_log.error(f"[MoveMongo] Export Logs Error : {e}")
            raise ServiceException("DB 로그 내보내기 실패", status_code=500) from e
