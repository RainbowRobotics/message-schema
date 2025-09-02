import os

from app.main import app
from dotenv import load_dotenv
from uvicorn import Config, Server

load_dotenv("config.env")

port = int(os.getenv("PORT") or 8001)

config = Config(app=app, host="0.0.0.0", port=port, lifespan="on")
server = Server(config)

if __name__ == "__main__":
    import asyncio
    asyncio.run(server.serve())
