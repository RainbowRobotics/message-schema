#!/bin/bash
set -e

if [ -z "$SERVICE" ]; then
  echo "❌ SERVICE 환경변수가 설정되지 않았습니다."
  exit 1
fi

ARCH=$(uname -m)

case "$ARCH" in
  x86_64)
    CONVERTED_ARCH="amd64"
    ;;
  aarch64)
    CONVERTED_ARCH="arm64"
    ;;
  arm64)
    CONVERTED_ARCH="arm64"
    ;;
  *)
    echo "❌ 지원하지 않는 아키텍처: $ARCH"
    exit 1
    ;;
esac


BIN_NAME="${SERVICE}.${CONVERTED_ARCH}.bin"
BIN_PATH="$WORKDIR/$BIN_NAME"

if [ ! -f "$BIN_PATH" ]; then
  echo "❌ 실행 파일이 없습니다: $BIN_PATH"
  exit 1
fi

echo "🚀 실행: $BIN_NAME"
chmod +x "$BIN_PATH"
exec "$BIN_PATH"