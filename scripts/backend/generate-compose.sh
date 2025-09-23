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

echo ""


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
      - IS_DEV=true
    restart: unless-stopped
    ports:
      - "${PORT}:${PORT}"
    depends_on:
      - zenoh-router
    env_file:
      - ${CONF_PATH}
    networks: [rb_net]
    volumes:
      - ../api-gateway/nginx.conf:/app/api-gateway/nginx.conf:ro
      - ./services/${NAME}:/app/backend/services/${NAME}
      - ./packages:/app/backend/packages
      - ./schemas:/app/backend/schemas

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
    ports:
      - "${PORT}:${PORT}"
    network_mode: host
    ipc: host
    env_file:
      - ${CONF_PATH}
    volumes:
      - ./services/${NAME}/${NAME}.${CONVERTED_ARCH}.bin:/${NAME}.bin

EOF
done

for FD in 3 4; do
  API_GATEWAY_CONTAINER_NAME="api-gateway"
  ZENOH_ROUTER_CONTAINER_NAME="zenoh-router"

  if [ "$FD" = "3" ]; then
    cat >&3 <<EOF
  api-gateway:
    build:
      context: ../api-gateway
      dockerfile: Dockerfile
    image: rrs-nginx:latest
    environment:
      - BUILDKIT_PROVENANCE=0
      - BUILDKIT_SBOM_SCAN=0
    container_name: api-gateway-dev
    restart: unless-stopped
    volumes:
      - ../api-gateway/nginx.conf:/etc/nginx/nginx.conf:ro
    ports:
      - "3000:3000"
    networks: [rb_net]
      
  zenoh-router:
    build:
      context: ../zenoh-router
      dockerfile: Dockerfile
    image: zenoh-router:latest
    container_name: ${ZENOH_ROUTER_CONTAINER_NAME}
    volumes:
      - ../zenoh-router/zenoh.json5:/etc/zenoh/zenoh.json5:ro
    ports:
      - "7447:7447/tcp"
      - "7446:7446/udp"
    networks: [rb_net]

  rrs-mongo-dev:
    image: mongo:7
    container_name: rrs-mongo-dev
    ports: ["27017:27017"]
    networks: [rb_net]
    volumes:
      - rrs-mongo-data:/data/db

volumes:
  rrs-mongo-data:

networks:
  rb_net:
    driver: bridge

EOF
  else
    cat >&4 <<EOF
  api-gateway:
    build:
      context: ../api-gateway
      dockerfile: Dockerfile
    image: rrs-nginx:latest
    environment:
      - BUILDKIT_PROVENANCE=0
      - BUILDKIT_SBOM_SCAN=0
    container_name: api-gateway-preview
    restart: unless-stopped
    volumes:
      - ../api-gateway/nginx.conf:/etc/nginx/nginx.conf:ro
    network_mode: host
    ipc: host

  rrs-mongo-preview:
    image: mongo:7
    container_name: rrs-mongo-preview
    ports: ["27017:27017"]
    network_mode: host
    volumes:
      - rrs-mongo-data:/data/db

volumes:
  rrs-mongo-data:
EOF
  fi
done


echo "✅ docker-compose.yml, docker-compose.prod.yml 생성 완료"
