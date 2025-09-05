from contextlib import asynccontextmanager  # noqa: I001

from fastapi import FastAPI

from app.api.move import router as move_router

from .zenoh_subs import zenoh_router

from schemas.rb.v1.state_fb import State  # noqa: F401


@asynccontextmanager
async def lifespan(app: FastAPI):
    print("📡 subscribe 진입 전", flush=True)
    await zenoh_router.startup()
    print("📡 subscribe 진입 후", flush=True)
    print(State.SizeOf(), flush=True)

    yield

    await zenoh_router.shutdown()
    print("⛔ zenoh 연결 종료")


app = FastAPI(lifespan=lifespan)


@app.get("/")
def welcome_root():
    return {"message": "Hello FastAPI"}


app.include_router(move_router)
