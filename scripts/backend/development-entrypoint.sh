#!/bin/bash
set -e

if [ -z "$SERVICE" ]; then
  echo "❌ SERVICE 환경변수가 설정되지 않았습니다."
  exit 1
fi

if [ -z "$PORT" ]; then
  echo "❌ PORT 환경변수가 설정되지 않았습니다."
  exit 1
fi

BACKEND_PATH="./backend"
APP_PATH="${BACKEND_PATH}/services/${SERVICE}"

cd "$BACKEND_PATH"

uv sync --all-packages --frozen

cd ../ && cd "$APP_PATH"

exec uv run uvicorn app.main:app --host 0.0.0.0 --port $PORT  --root-path=/$SERVICE --reload --reload-dir "../../" --reload-include '**/*.py'
