from fastapi import APIRouter

amr_network_router = APIRouter(tags=["AMR 네트워크"])

@amr_network_router.post("/test/network")
async def test_network_command():
    return {"message": "Hello, World!"} 