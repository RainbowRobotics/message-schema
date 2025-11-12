from fastapi import APIRouter

amr_localization_router = APIRouter(tags=["AMR 위치 추정"])

@amr_localization_router.post("/test/localization")
async def test_localization_command():
    return {"message": "Hello, World!"} 