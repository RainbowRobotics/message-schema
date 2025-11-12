from fastapi import APIRouter

amr_update_router = APIRouter(tags=["AMR 업데이트"])

@amr_update_router.post("/test/update")
async def test_update_command():
    return {"message": "Hello, World!"} 