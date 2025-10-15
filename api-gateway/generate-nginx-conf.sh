#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$SCRIPT_DIR/.."
TEMPLATE="$SCRIPT_DIR/nginx.template.conf"
OUTPUT="$SCRIPT_DIR/nginx.conf"
SERVICES_DIR="$REPO_ROOT/backend/services"
DEV_MODE=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --dev*)
      DEV_MODE=true
      shift
      ;;
  esac
done


SERVICE_BLOCKS=""

if [ "$DEV_MODE" = true ]; then
  OUTPUT="$SCRIPT_DIR/nginx.dev.conf"
fi

for service_path in "$SERVICES_DIR"/*; do
  echo "service_path: $service_path"
  [ -d "$service_path" ] || continue

  CONF="$service_path/config.env"

  [ -f "$CONF" ] || continue

  SERVICE_NAME=$(grep -E '^SERVICE_NAME=' "$CONF" | cut -d= -f2 | tr -d '\r')
  PORT=$(grep -E '^PORT=' "$CONF" | cut -d= -f2 | tr -d '\r')

  [ -n "$SERVICE_NAME" ] || continue
  [ -n "$PORT" ] || continue

  if [ "$DEV_MODE" = true ]; then
    PROXY_PASS="http://${SERVICE_NAME}:${PORT}"
  else
    PROXY_PASS="http://127.0.0.1:${PORT}"
  fi

  LOCATION_PREFIX="$SERVICE_NAME"

  SERVICE_BLOCKS+="
  
        set \$${LOCATION_PREFIX}_upstream ${PROXY_PASS};
    
        location ^~ /${LOCATION_PREFIX}/ {
            rewrite ^/${LOCATION_PREFIX}/(.*)$ /\$1 break;
            proxy_pass \$${LOCATION_PREFIX}_upstream;

            proxy_http_version 1.1;
            proxy_set_header Upgrade \$http_upgrade;
            proxy_set_header Connection \$connection_upgrade;
            proxy_set_header Host \$host;
            proxy_set_header X-Real-IP \$remote_addr;
            proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto \$scheme;

            proxy_read_timeout 120s;
            proxy_send_timeout 120s;

            proxy_buffering off;
        }"
  
done

{
  while IFS= read -r line || [ -n "$line" ]; do
    if echo "$line" | grep -q '{{SERVICE_BLOCKS}}'; then
      printf "%s\n" "$SERVICE_BLOCKS"
    else
      printf "%s\n" "$line"
    fi
  done < "$TEMPLATE"
} > "$OUTPUT"

echo "✅ nginx.conf 생성 완료"
