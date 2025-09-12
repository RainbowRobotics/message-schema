#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BE="$ROOT/backend"

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

OUT_DEV="backend/docker-compose.yml"
OUT_PROD="backend/docker-compose.preview.yml"

exec 3> "$OUT_DEV"
exec 4> "$OUT_PROD"

# 공통 헤더
echo "version: '3.8'" >&3
echo "services:" >&3
echo "" >&4
echo "services:" >&4

for dir in "$BE/services"/*; do
  [ -d "$dir" ] || continue
  CONF="$dir/config.env"
  [ -f "$CONF" ] || continue

  NAME=$(grep '^SERVICE_NAME=' "$CONF" | cut -d= -f2 | tr -d ' \r')
  PORT=$(grep '^PORT=' "$CONF" | cut -d= -f2 | tr -d ' \r')

  [ -n "$NAME" ] || continue
  [ -n "$PORT" ] || continue

  CONF_PATH="./services/${NAME}/config.env"

  # 개발용
  cat >&3 <<EOF
  ${NAME}:
    build:
      context: ../
      dockerfile: backend/Dockerfile.dev
    image: rrs-fastapi-dev:latest
    container_name: ${NAME}-service-dev
    environment:
      - SERVICE=${NAME}
      - PORT=${PORT}
    restart: unless-stopped
    network_mode: host
    ipc: host
    env_file:
      - ${CONF_PATH}
    volumes:
      - ../api-gateway/nginx.conf:/app/api-gateway/nginx.conf:ro
      - ./services/${NAME}:/app/backend/services/${NAME}
      - ./packages:/app/backend/packages

EOF

  # 운영 preview 용
  cat >&4 <<EOF
  ${NAME}:
    build:
      context: ../
      dockerfile: backend/Dockerfile.prod
    image: rrs-fastapi-preview:latest
    container_name: ${NAME}-service-preview
    environment:
      - SERVICE=${NAME}
      - PORT=${PORT}
    restart: unless-stopped
    env_file:
      - ${CONF_PATH}
    volumes:
      - ./services/${NAME}/${NAME}.${CONVERTED_ARCH}.bin:/${NAME}.bin
    network_mode: host
    ipc: host

EOF
done

# api-gateway, zenoh-router 공통 처리
for FD in 3 4; do
  if [ "$FD" -eq 3 ]; then
    cat >&$FD <<EOF
  api-gateway:
    build:
      context: ../api-gateway
      dockerfile: Dockerfile
    image: rrs-nginx:latest
    environment:
      - BUILDKIT_PROVENANCE=0
      - BUILDKIT_SBOM_SCAN=0
    container_name: api-gateway
    restart: unless-stopped
    volumes:
      - ../api-gateway/nginx.conf:/etc/nginx/nginx.conf:ro
    network_mode: host
EOF
  else
    cat >&$FD <<EOF
  api-gateway:
    build:
      context: ../api-gateway
      dockerfile: Dockerfile
    image: rrs-nginx:latest
    environment:
      - BUILDKIT_PROVENANCE=0
      - BUILDKIT_SBOM_SCAN=0
    container_name: api-gateway
    restart: unless-stopped
    volumes:
      - ../api-gateway/nginx.conf:/etc/nginx/nginx.conf:ro
    network_mode: host
EOF
  fi
done

echo "✅ docker-compose.yml, docker-compose.prod.yml 생성 완료"
