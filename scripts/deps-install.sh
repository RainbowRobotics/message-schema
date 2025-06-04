#!/bin/bash
set -e

BRANCH="develop"
VERSION="20250604124828"
BASE_URL="https://github.com/rainbow-mobile/rainbow-release-apps/raw/refs/heads/${BRANCH}/slamnav2/${VERSION}"
PART_PREFIX="lib-part"
INDEX=1

echo "🌐 [rainbow-release-app]레포에서 ${PART_PREFIX}-*.tar.gz 파일 다운로드 및 압축 해제 시작..."

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/lib"

if [[ -d "$LIB_DIR" ]]; then
  rm -rf "$LIB_DIR"
fi
mkdir -p "$LIB_DIR"

while :; do
  FILENAME="${PART_PREFIX}-${INDEX}.tar.gz"
  URL="${BASE_URL}/${FILENAME}"
  DEST="${LIB_DIR}/${FILENAME}"

  echo "🔎 다운로드 시도: $URL"

  HTTP_CODE=$(curl -s -o "$DEST" -w "%{http_code}" "$URL")
  if [ "$HTTP_CODE" -eq 404 ]; then
    echo "❌ 파일이 존재하지 않습니다: $FILENAME"
    break
  fi

  echo "🗃️ 압축 해제: $FILENAME → $LIB_DIR"
  tar -xzf "$DEST" -C "$LIB_DIR"

  ((INDEX++))
done

echo "✅ 다운로드 및 압축 해제 완료 (${INDEX}개 시도됨)"
