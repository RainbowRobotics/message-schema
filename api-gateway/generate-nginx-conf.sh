#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$SCRIPT_DIR/.."
TEMPLATE="$SCRIPT_DIR/nginx.template.conf"
OUTPUT="$SCRIPT_DIR/nginx.conf"
SERVICES_DIR="$REPO_ROOT/backend/services"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dev)
      DEV_MODE=true
      shift
      ;;
  esac
done

SERVICE_BLOCKS=""

for service_path in "$SERVICES_DIR"/*; do
  [ -d "$service_path" ] || continue

  CONF="$service_path/config.env"
  RUN_PY="$service_path/run.py"

  [ -f "$CONF" ] || continue
  [ -f "$RUN_PY" ] || continue

  SERVICE_NAME=$(grep -E '^SERVICE_NAME=' "$CONF" | cut -d= -f2 | tr -d '\r')
  PORT=$(grep -E '^PORT=' "$CONF" | cut -d= -f2 | tr -d '\r')

  [ -n "$SERVICE_NAME" ] || continue
  [ -n "$PORT" ] || continue

  if [ "$DEV_MODE" = true ]; then
    SERVICE_BLOCKS="${SERVICE_BLOCKS}
        location /${SERVICE_NAME}/ {            
            proxy_pass http://${SERVICE_NAME}:8000/;
            proxy_http_version 1.1;
            proxy_set_header Upgrade \$http_upgrade;
            proxy_set_header Connection "upgrade";
            proxy_set_header Host \$host;
            proxy_set_header X-Real-IP \$remote_addr;
            proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        }"
  else
    SERVICE_BLOCKS="${SERVICE_BLOCKS}
        location /${SERVICE_NAME}/ {
            proxy_pass http://127.0.0.1:${PORT}/;
            proxy_http_version 1.1;
            proxy_set_header Upgrade \$http_upgrade;
            proxy_set_header Connection "upgrade";
            proxy_set_header Host \$host;
            proxy_set_header X-Real-IP \$remote_addr;
            proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        }"
  fi
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
