#!/bin/bash

# host 서비스 경로 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_DIR="$ROOT/backend/host"

# config.env 파일 읽고 포트 추출
CONF="$HOST_DIR/config.env"
[ -f "$CONF" ] || continue
NAME=$(grep '^SERVICE_NAME=' "$CONF" | cut -d= -f2 | tr -d ' \r')
PORT=$(grep '^PORT=' "$CONF" | cut -d= -f2 | tr -d ' \r')
DATA_DIR=$(grep '^DATA_DIR=' "$CONF" | cut -d= -f2 | tr -d ' \r') || true

if [ -z "$PORT" ]; then
  echo "❌ PORT 환경변수가 설정되지 않았습니다."
  exit 1
fi

# host 서비스 실행
cd $HOST_DIR
exec uv run uvicorn app.main:app --host 0.0.0.0 --port $PORT --reload --reload-dir "../" --reload-include '**/*.py'

# exec uv run --project . uvicorn run:app --host 0.0.0.0 --port $PORT
