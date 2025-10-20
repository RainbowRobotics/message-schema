import asyncio

from motor.motor_asyncio import AsyncIOMotorCollection
from rb_database.mongo_db import MongoDB
from rb_resources.file import read_json_file

from .info_schema import RobotInfo


class InfoService:
    def __init__(self):
        self._cache_robot_info = {}

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
