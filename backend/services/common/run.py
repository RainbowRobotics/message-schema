import multiprocessing as mp
import os
import sys

if __name__ == "__main__":
    mp.freeze_support()

    # 환경변수로 워커 프로세스 체크
    if os.environ.get("_PYINSTALLER_WORKER") == "1":
        print(f"[Worker {os.getpid()}] Detected, exiting main", flush=True)
        sys.exit(0)

    # sys.argv로 체크 (spawn이 --multiprocessing-fork 추가)
    is_mp_worker = any("multiprocessing-fork" in str(arg) for arg in sys.argv)
    if is_mp_worker:
        print(f"[Worker {os.getpid()}] Detected via argv, exiting main", flush=True)
        sys.exit(0)

    print(f"[Main {os.getpid()}] Starting FastAPI app", flush=True)

    # path 설정
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    # 환경변수 로드
    from dotenv import load_dotenv
    load_dotenv("config.env")

    port = int(os.getenv("PORT") or 11337)

    # FastAPI 앱 실행
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
