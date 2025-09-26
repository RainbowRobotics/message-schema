import asyncio

from motor.motor_asyncio import AsyncIOMotorCollection


class InfoService:
    def __init__(self):
        self._cache_robot_info = {}

    async def get_info(self):
        pass

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
