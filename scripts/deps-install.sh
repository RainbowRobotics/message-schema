#!/bin/bash
set -e

BRANCH=main
VERSION=version
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
  URL="${BASE_URL}/lib/${FILENAME}"
  DEST="${SCRIPT_DIR}/${FILENAME}"

  echo "🔎 다운로드 시도: $URL"
  curl -fLo "$DEST" "$URL" 2>/dev/null || break

  echo "🗃️ 압축 해제: $FILENAME → $LIB_DIR"
  tar -xzf "$DEST" -C "$LIB_DIR"

  ((INDEX++))
done

echo "✅ 다운로드 및 압축 해제 완료 (${INDEX}개 시도됨)"
