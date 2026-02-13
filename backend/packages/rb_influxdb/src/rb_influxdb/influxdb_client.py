"""
influxdb_client.py
"""
import os
from typing import (
    Annotated,
)

from fastapi import (
    Depends,
    FastAPI,
)
from influxdb_client import (
    InfluxDBClient,
)
from influxdb_client.client.write_api import (
    ASYNCHRONOUS,
    WriteApi,
)
from rb_modules.log import (
    rb_log,
)

client: InfluxDBClient | None = None
write_api: WriteApi | None = None


async def init_influxdb(
    app: FastAPI,
    url: str | None = None,
    token: str | None = None,
    org: str | None = None,
    bucket: str | None = None,
    timeout: int = 10000,
):
    """
    [InfluxDB 초기화]
    - 비동기 함수로 처리
    - 데이터베이스 초기화
    """
    global client, write_api

    url = url or os.getenv("INFLUXDB_URL", "http://rrs-influxdb-dev:8086")
    token = token or os.getenv("INFLUXDB_TOKEN", "J4ecotdoPLl9gctrtN6SjFPG0s75e6z3UeIMkBBKQJXZvLs-UEDvpzmrzyOpDBEe6COvPtYwry6Ik8hWn410UA==")
    org = org or os.getenv("INFLUXDB_ORG", "rainbow")
    bucket = bucket or os.getenv("INFLUXDB_BUCKET", "rrs")

    rb_log.info(f"[InfluxDB] Initializing with url={url}, org={org}, bucket={bucket}, token={token}")

    try:
        client = InfluxDBClient(url=url, token=token, org=org, timeout=timeout)
        write_api = client.write_api(write_options=ASYNCHRONOUS)

        # 연결 테스트
        health = client.health()
        if health.status == "pass":
            rb_log.info(f"[InfluxDB] Connected to {url}, org={org}, bucket={bucket}")
        else:
            rb_log.warning(f"[InfluxDB] Health check failed: {health.message}")

        app.state.influxdb_client = client
        app.state.influxdb_write_api = write_api
        app.state.influxdb_org = org
        app.state.influxdb_bucket = bucket

    except Exception as e:
        rb_log.error(f"[InfluxDB] Initialization failed: {e}")
        raise


async def close_db(app: FastAPI):
    """
    [InfluxDB 종료]
    - 비동기 함수로 처리
    - 데이터베이스 종료
    """
    w = app.state.influxdb_write_api if hasattr(app.state, "influxdb_write_api") else None
    c = app.state.influxdb_client if hasattr(app.state, "influxdb_client") else None

    if w:
        try:
            w.close()
        except Exception as e:  # noqa: BLE001
            rb_log.error(f"[InfluxDB] WriteApi close error: {e}")

    if c:
        try:
            c.close()
        except Exception as e:  # noqa: BLE001
            rb_log.error(f"[InfluxDB] Client close error: {e}")


async def wait_db_ready(timeout: int = 15):
    """
    [InfluxDB 준비 대기]
    - 비동기 함수로 처리
    - 데이터베이스 준비 대기
    - 데이터베이스가 준비되지 않으면 예외 발생
    - timeout: 최대 대기 시간 (기본 15초)
    """
    import asyncio
    import time

    start = time.monotonic()
    while client is None:
        rb_log.info("🔎 wait InfluxDB ready")
        if time.monotonic() - start > timeout:
            raise RuntimeError("InfluxDB not ready")
        await asyncio.sleep(0.05)


def get_client() -> InfluxDBClient:
    """
    [InfluxDB 클라이언트 조회]
    - InfluxDB 클라이언트 반환
    - 데이터베이스가 준비되지 않으면 예외 발생
    """
    if client is None:
        raise RuntimeError("InfluxDB client not initialized. Call init_db() first.")
    return client


def get_write_api() -> WriteApi:
    """
    [InfluxDB Write API 조회]
    - InfluxDB Write API 반환
    - 데이터베이스가 준비되지 않으면 예외 발생
    """
    if write_api is None:
        raise RuntimeError("InfluxDB write_api not initialized. Call init_db() first.")
    return write_api


InfluxDBClientDep = Annotated[InfluxDBClient, Depends(get_client)]
InfluxDBWriteApiDep = Annotated[WriteApi, Depends(get_write_api)]
