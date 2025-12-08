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

APP_PATH="./services/${SERVICE}"

cd "$APP_PATH"

exec uv run uvicorn app.main:app --host 0.0.0.0 --port $PORT --reload --reload-dir "../../" --reload-include '**/*.py'

# exec uv run uvicorn app.main:app --host 0.0.0.0 --port $PORT  --reload-dir "../../" --reload-include '**/*.py'
