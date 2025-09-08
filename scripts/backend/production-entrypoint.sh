#!/bin/bash
set -e

if [ -z "$SERVICE" ]; then
  echo "❌ SERVICE 환경변수가 설정되지 않았습니다."
  exit 1
fi

BIN_NAME="$SERVICE.bin"
BIN_PATH="$WORKDIR/$BIN_NAME"

if [ ! -f "$BIN_PATH" ]; then
  echo "❌ 실행 파일이 없습니다: $BIN_PATH"
  exit 1
fi

echo "🚀 실행: $BIN_NAME"
chmod +x "$BIN_PATH"
exec "$BIN_PATH"