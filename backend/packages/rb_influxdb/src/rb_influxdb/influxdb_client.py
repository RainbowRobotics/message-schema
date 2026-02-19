"""
influxdb_client.py
"""
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
from influxdb_client.client.write.point import Point
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
    timeout: int = 10000,
):
    """
    [InfluxDB 초기화]
    - 비동기 함수로 처리
    - 데이터베이스 초기화
    """
    global client, write_api

    rb_log.info(f"[InfluxDB] Initializing with url={url}, org={org}")

    try:
        if not url:
            raise ValueError("url is required")
        if not token:
            raise ValueError("token is required")
        if not org:
            raise ValueError("org is required")

        client = InfluxDBClient(url=url, token=token, org=org, timeout=timeout)
        write_api = client.write_api(write_options=ASYNCHRONOUS)

        # 연결 테스트
        health = client.health()
        if health.status == "pass":
            rb_log.info(f"[InfluxDB] Connected to {url}, org={org}")
        else:
            rb_log.warning(f"[InfluxDB] Health check failed: {health.message}")

        app.state.influxdb_client = client
        app.state.influxdb_write_api = write_api
        app.state.influxdb_org = org

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


def write_point(org: str, bucket: str, point: Point):
    """
    [InfluxDB Point 저장]
    - InfluxDB Point 저장
    - 데이터베이스가 준비되지 않으면 예외 발생
    """
    if client is None:
        raise RuntimeError("InfluxDB client not initialized. Call init_db() first.")
    if write_api is None:
        raise RuntimeError("InfluxDB write_api not initialized. Call init_db() first.")
    write_api.write(bucket=bucket, org=org, record=point)

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
