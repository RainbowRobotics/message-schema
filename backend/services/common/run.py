# run.py
import multiprocessing as mp
import os
import sys

if __name__ == "__main__":
    mp.freeze_support()

    # ✅ ScriptExecutor의 spawn 자식 프로세스만 차단
    # multiprocessing-fork 인자는 spawn이 자동으로 추가함
    is_mp_worker = any("multiprocessing-fork" in str(arg) for arg in sys.argv)

    if is_mp_worker:
        # spawn 워커 프로세스는 여기서 대기
        print(f"MP Worker {os.getpid()} ready", flush=True)
        # multiprocessing이 알아서 처리
        # 명시적으로 종료하지 말고 대기
        import time

        while True:
            time.sleep(1000)

# ✅ 메인 프로세스만 앱 시작
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dotenv import load_dotenv

load_dotenv("config.env")

port = int(os.getenv("PORT") or 11337)

if __name__ == "__main__":
    print(f"Main process {os.getpid()} starting FastAPI", flush=True)

    import asyncio

    try:
        from app.main import app
        from uvicorn import Config, Server

        config = Config(app=app, host="0.0.0.0", port=port, lifespan="on")
        server = Server(config)


        asyncio.run(server.serve())
    except ImportError as e:
        print(f"Import failed: {e}", file=sys.stderr)
        sys.exit(1)
