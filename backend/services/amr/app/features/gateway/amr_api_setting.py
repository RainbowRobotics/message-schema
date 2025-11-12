from fastapi import APIRouter

amr_setting_router = APIRouter(tags=["AMR 설정"])

@amr_setting_router.post("/test/setting")
async def test_setting_command():
    return {"message": "Hello, World!"} 