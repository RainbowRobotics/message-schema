import os
import sys

from app.main import app
from dotenv import load_dotenv
from uvicorn import (
    Config,
    Server,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

load_dotenv("config.env")

port = int(os.getenv("PORT") or 8010)

print(f"Starting Host Service on port {port}")
config = Config(app=app, host="0.0.0.0", port=port, lifespan="on")
server = Server(config)

if __name__ == "__main__":
    import asyncio

    asyncio.run(server.serve())
