#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BE="$ROOT/backend"
DOCKER_USER="${DOCKER_USER-}"

USER_LINE=""
if [[ -n "${DOCKER_USER}" ]]; then
  USER_LINE="    user: \"${DOCKER_USER}\""
fi

OUT_DEV="backend/docker-compose.dev.yml"
OUT_PREVIEW="backend/docker-compose.preview.yml"
OUT_PROD="backend/docker-compose.yml"

exec 3> "$OUT_DEV"
exec 4> "$OUT_PREVIEW"
exec 5> "$OUT_PROD"

# 공통 헤더
echo "version: '3.8'" >&3
echo "services:" >&3
echo "" >&4
echo "services:" >&4
echo "" >&5
echo "services:" >&5


for dir in "$BE/services"/*; do
  [ -d "$dir" ] || continue
  CONF="$dir/config.env"
  [ -f "$CONF" ] || continue

  echo "CONF: $CONF"

  NAME=$(grep '^SERVICE_NAME=' "$CONF" | cut -d= -f2 | tr -d ' \r')
  PORT=$(grep '^PORT=' "$CONF" | cut -d= -f2 | tr -d ' \r')
  DATA_DIR=$(grep '^DATA_DIR=' "$CONF" | cut -d= -f2 | tr -d ' \r') || true

  [ -n "$NAME" ] || continue
  [ -n "$PORT" ] || continue
  if [ -z "$DATA_DIR" ]; then
    DATA_DIR="./data"
  fi

  CONF_PATH="./services/${NAME}/config.env"
  COMMON_ENV_PATH="./.env"
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
      - PYTHONDONTWRITEBYTECODE=1
      - IS_DEV=true
    restart: unless-stopped
    ports:
      - "${PORT}:${PORT}"
    depends_on:
      - zenoh-router
    env_file:
      - ${CONF_PATH}
      - ${COMMON_ENV_PATH}
    networks: [rb_net]
    volumes:
      - ../api-gateway/nginx.conf:/app/api-gateway/nginx.conf:ro
      - ./services/${NAME}:/app/backend/services/${NAME}
      - ./packages:/app/backend/packages
      - ./schemas:/app/backend/schemas
      - ./documents:/app/backend/documents
      - ${DATA_DIR}:/app/data
EOF

  # preview 용
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
      - INFLUXDB_INIT_PASSWORD_FILE=/run/secrets/influxdb_password
      - INFLUXDB_INIT_ADMIN_TOKEN_FILE=/run/secrets/influxdb_token
    secrets:
      - influxdb_password
      - influxdb_token
    restart: unless-stopped
    ports:
      - "${PORT}:${PORT}"
    network_mode: host
    ipc: host
    env_file:
      - ${CONF_PATH}
    volumes:
      - ./services/${NAME}/${NAME}.arm64.bin:/${NAME}.arm64.bin
      - ./services/${NAME}/${NAME}.amd64.bin:/${NAME}.amd64.bin
      - ${DATA_DIR}:/app/data

EOF

  # prod 용
  cat >&5 <<EOF
  ${NAME}:
    build:
      dockerfile: Dockerfile.prod
    image: rrs-fastapi:latest
    container_name: ${NAME}-service
    profiles: [${NAME}, all]
    environment:
      - SERVICE=${NAME}
      - PORT=${PORT}
      - INFLUXDB_INIT_PASSWORD_FILE=/run/secrets/influxdb_password
      - INFLUXDB_INIT_ADMIN_TOKEN_FILE=/run/secrets/influxdb_token
    secrets:
      - influxdb_password
      - influxdb_token
    restart: unless-stopped
    ports:
      - "${PORT}:${PORT}"
    network_mode: host
    ipc: host
    env_file:
      - ${CONF_PATH}
      - ${COMMON_ENV_PATH}
    volumes:
      - ./services/${NAME}/${NAME}.arm64.bin:/${NAME}.arm64.bin
      - ./services/${NAME}/${NAME}.amd64.bin:/${NAME}.amd64.bin
      - ${DATA_DIR}:/app/data
      - ./documents:/app/backend/documents
EOF
done

for FD in 3 4 5; do
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
      - ../api-gateway/nginx.dev.conf:/etc/nginx/nginx.conf:ro
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
      - ../zenoh-router/zenoh.json5:/etc/zenoh/config.json5:ro
    ports:
      - "7447:7447/tcp"
      - "7446:7446/udp"
    networks: [rb_net]

  rrs-mongo-dev:
    image: mongo:7
    container_name: rrs-mongo-dev
    command: ["mongod", "--replSet", "rs0", "--bind_ip_all"]
    ports: ["27017:27017"]
    networks: [rb_net]
    volumes:
      - rrs-mongo-data:/data/db
      - ../scripts/backend/mongo-dev-init.js:/docker-entrypoint-initdb.d/mongo-init.js:ro

  rrs-influxdb-dev:
    image: influxdb:2.7.1
    container_name: rrs-influxdb-dev
    ports: ["8086:8086"]
    networks: [rb_net]
    environment:
      - DOCKER_INFLUXDB_INIT_MODE=setup
      - DOCKER_INFLUXDB_INIT_USERNAME=rainbow
      - DOCKER_INFLUXDB_INIT_PASSWORD=rainbow2011
      - DOCKER_INFLUXDB_INIT_ORG=rrs
      - DOCKER_INFLUXDB_INIT_BUCKET=rrs-rt
      - DOCKER_INFLUXDB_INIT_RETENTION=7d
      - DOCKER_INFLUXDB_INIT_ADMIN_TOKEN=rainbowToken
    volumes:
      - rrs-influxdb-data:/var/lib/influxdb2
      - rrs-influxdb-config:/etc/influxdb2

volumes:
  rrs-mongo-data:
  rrs-influxdb-data:
  rrs-influxdb-config:

networks:
  rb_net:
    driver: bridge

EOF
  elif [ "$FD" = "4" ]; then
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
      - ../api-gateway/nginx.dev.conf:/etc/nginx/nginx.conf:ro
    network_mode: host

  rrs-influxdb-preview:
    image: influxdb:2.7.1
    container_name: rrs-influxdb-preview
    ports: ["8086:8086"]
    networks: [rb_net]
    environment:
      - DOCKER_INFLUXDB_INIT_MODE=setup
      - DOCKER_INFLUXDB_INIT_USERNAME=rainbow
      - DOCKER_INFLUXDB_INIT_PASSWORD_FILE=/run/secrets/influxdb_password
      - DOCKER_INFLUXDB_INIT_ORG=rrs
      - DOCKER_INFLUXDB_INIT_BUCKET=rrs-rt
      - DOCKER_INFLUXDB_INIT_RETENTION=7d
      - DOCKER_INFLUXDB_INIT_ADMIN_TOKEN_FILE=/run/secrets/influxdb_token
    secrets:
      - influxdb_password
      - influxdb_token
    volumes:
      - rrs-influxdb-data:/var/lib/influxdb2
      - rrs-influxdb-config:/etc/influxdb2

volumes:
  rrs-mongo-data:
  rrs-influxdb-data:
  rrs-influxdb-config:

secrets:
  influxdb_password:
    file: ./secrets/influxdb_password
  influxdb_token:
    file: ./secrets/influxdb_token

networks:
  rb_net:
    driver: bridge
EOF
  else
    cat >&5 <<EOF
  api-gateway:
    build:
      context: ./api-gateway
      dockerfile: Dockerfile
    image: rrs-nginx:latest
    profiles: [api-gateway]
    environment:
      - BUILDKIT_PROVENANCE=0
      - BUILDKIT_SBOM_SCAN=0
    container_name: api-gateway
    restart: unless-stopped
    volumes:
      - ./api-gateway/nginx.conf:/etc/nginx/nginx.conf:ro
    network_mode: host

  rrs-mongo:
    image: mongo:7
    container_name: rrs-mongo
    command: ["mongod", "--replSet", "rs0", "--bind_ip_all"]
    profiles: [rrs-mongo]
    ports: ["27017:27017"]
    network_mode: host
    volumes:
      - rrs-mongo-data:/data/db

  rrs-influxdb:
    image: influxdb:2.7.1
    container_name: rrs-influxdb
    ports: ["8086:8086"]
    networks: [rb_net]
    environment:
      - DOCKER_INFLUXDB_INIT_MODE=setup
      - DOCKER_INFLUXDB_INIT_USERNAME=rainbow
      - DOCKER_INFLUXDB_INIT_PASSWORD_FILE=/run/secrets/influxdb_password
      - DOCKER_INFLUXDB_INIT_ORG=rrs
      - DOCKER_INFLUXDB_INIT_BUCKET=rrs-rt
      - DOCKER_INFLUXDB_INIT_RETENTION=7d
      - DOCKER_INFLUXDB_INIT_ADMIN_TOKEN_FILE=/run/secrets/influxdb_token
    secrets:
      - influxdb_password
      - influxdb_token
    volumes:
      - rrs-influxdb-data:/var/lib/influxdb2
      - rrs-influxdb-config:/etc/influxdb2

volumes:
  rrs-mongo-data:
  rrs-influxdb-data:
  rrs-influxdb-config:

secrets:
  influxdb_password:
    file: ./secrets/influxdb_password
  influxdb_token:
    file: ./secrets/influxdb_token

networks:
  rb_net:
    driver: bridge
EOF
  fi
done


echo "✅ docker-compose.dev.yml, docker-compose.preview.yml, docker-compose.yml 생성 완료"
