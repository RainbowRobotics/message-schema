from fastapi import APIRouter

amr_map_router = APIRouter(tags=["AMR 지도"])

@amr_map_router.post("/test/map")
async def test_map_command():
    return {"message": "Hello, World!"} 