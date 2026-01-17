import asyncio

from fastapi import HTTPException
from motor.motor_asyncio import AsyncIOMotorCollection
from pymongo import ReturnDocument
from rb_database.mongo_db import MongoDB
from rb_resources.file import read_json_file
from rb_utils.parser import t_to_dict

from .info_schema import Request_Upsert_Robot_InfoPD, RobotInfo


class InfoService:
    def __init__(self):
        self._cache_robot_info = {}

    async def get_robot_category_list(self, *, db: MongoDB):
        """robot_models.json 파일에서 모든 robot_model의 be_service를 중복 제거하여 리스트로 반환한다."""

        robot_models = read_json_file("data", "robot_models.json")

        be_services = set()
        for model_details in robot_models.values():
            be_service = model_details.get("be_service")
            if be_service:
                be_services.add(be_service)

        return list(be_services)
        # pass

    async def get_robot_component_list(self, *, be_service: str | None = None, db: MongoDB):
        """
        robot_models.json 파일에서 모든 robot_model을 조회하여 반환한다.
        be_service가 있으면 be_service에 해당하는 모든 component를 조회하여 반환한다.
        """

        robot_models = read_json_file("data", "robot_models.json")
        all_components = []

        for model_details in robot_models.values():
            model_service = model_details.get("be_service")

            if be_service is None or (
                model_service and be_service.casefold() in model_service.casefold()
            ):
                all_components.append(model_details.get("model"))

        return list(all_components)

    async def get_robot_info(self, *, db: MongoDB):
        robot_models = read_json_file("data", "robot_models.json")

        if "robot_info" not in await db.list_collection_names():
            await db.create_collection("robot_info")

        doc = await db["robot_info"].find_one({}, {"_id": 0})

        if not doc:
            doc = dict(RobotInfo())

        robot_model_key = doc.get("robotModel")

        model_info = robot_models.get(robot_model_key)

        return {"info": doc, "modelInfo": model_info}

    async def upsert_robot_info(self, *, db: MongoDB, request: Request_Upsert_Robot_InfoPD):
        robot_info_col = db["robot_info"]

        existing = await robot_info_col.find_one({})

        if existing is None:
            # === INSERT ===
            insert_doc = RobotInfo.model_validate(t_to_dict(request)).model_dump(
                exclude_none=True,
            )

            if not insert_doc["robotModel"]:
                raise HTTPException(400, "robotModel is required")

            robot_models = read_json_file("data", "robot_models.json")

            model_info = robot_models.get(insert_doc["robotModel"])

            if not model_info:
                raise HTTPException(400, "Invalid robotModel")

            insert_doc["components"] = model_info["components"]

            res = await robot_info_col.find_one_and_update(
                {},
                {"$set": insert_doc},
                upsert=True,
                return_document=ReturnDocument.AFTER,
            )

        else:
            # === UPDATE ===
            patch_doc = (
                request.model_dump(exclude_none=True, exclude_unset=True)
                if hasattr(request, "model_dump")
                else t_to_dict(request)
            )

            if patch_doc.get("robotModel"):
                robot_models = read_json_file("data", "robot_models.json")

                model_info = robot_models.get(patch_doc["robotModel"])

                if not model_info:
                    raise HTTPException(400, "Invalid robotModel")

                patch_doc["components"] = model_info["components"]

                if existing["robotModel"] != patch_doc["robotModel"]:
                    patch_doc["programId"] = None

            # patch_doc 에 들어있는 필드만 업데이트됨
            # 기존 DB 값은 손대지 않는다.
            res = await robot_info_col.find_one_and_update(
                {"_id": existing["_id"]},
                {"$set": patch_doc},
                return_document=ReturnDocument.AFTER,
            )

        if res:
            res.pop("_id", None)

        return res

    async def get_cache_robot_info(self):
        return self._cache_robot_info

    async def load_robot_info(self, col: AsyncIOMotorCollection):
        doc = await col.find_one({}, {"_id": 0}) or {}

        return doc

    async def watch_robot_info(self, col: AsyncIOMotorCollection):
        self._cache_robot_info = await self.load_robot_info(col)

        try:
            async with col.watch(
                [{"$match": {"operationType": {"$in": ["insert", "update", "replace"]}}}]
            ) as stream:
                async for _ in stream:
                    self._cache_robot_info = await self.load_robot_info(col)
        except Exception:
            await asyncio.sleep(1)
            asyncio.create_task(self.watch_robot_info(col))

    async def get_robot_urdf_link_map(self, robot_model: str):
        robot_models = read_json_file("data", "robot_models.json")
        urdf_link_map = read_json_file("data", "urdf_link_map.json").get(robot_model)

        if not urdf_link_map:
            return None

        components = urdf_link_map.get("components") or {}
        new_components = {}

        for name, comp in components.items():
            be_service = robot_models.get(name).get("be_service")
            comp["beService"] = be_service

            new_components[name] = comp

        return {**urdf_link_map, "components": new_components}
