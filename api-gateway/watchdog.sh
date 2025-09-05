#!/bin/bash

NGINX_CONF="/etc/nginx/nginx.conf"
TEMP_FILE="/tmp/nginx.conf.tmp"

cleanup() {
  echo "🧹 임시 파일 정리 중..."
  rm -f "$TEMP_FILE"
  exit 0
}

trap cleanup SIGTERM SIGINT

cp "$NGINX_CONF" "$TEMP_FILE"

inotifywait -m -e modify "$NGINX_CONF" | while read -r path event file; do  
  if ! cmp -s "$NGINX_CONF" "$TEMP_FILE"; then
    echo "✅ 내용이 실제로 변경됨 → nginx reload"
    nginx -s reload

    cp "$NGINX_CONF" "$TEMP_FILE"
  else
    echo "⏩ 파일은 수정되었지만 내용은 동일함 → reload 건너뜀"
  fi
done

cleanup