from pydantic import BaseModel


class SocketConfig(BaseModel):
    namespace: str = "/robot"
    logger: bool = False
    cors_origins: list[str] | str = "*"
    ping_interval: int = 1
    ping_timeout: int = 60
    allow_upgrades: bool = True

    # 멀티 워커/멀티 인스턴스 확장 시 Redis 사용(단일 인스턴스면 None)
    redis_url: str | None = None

    # JWT
    # jwt_secret: str = "change-me"
    # jwt_alg: str = "HS256"
    # jwt_aud: str | None = None
    # jwt_iss: str | None = None
