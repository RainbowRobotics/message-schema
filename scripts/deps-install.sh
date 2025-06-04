#!/bin/bash
set -e

BRANCH=main
VERSION=version
GIT_BASE_URL="https://github.com/rainbow-mobile/rainbow-release-apps/raw/refs/heads/${BRANCH}/slamnav2/${VERSION}"
S3_BASE_URL="https://rainbow-deploy.s3.ap-northeast-2.amazonaws.com/slamnav2/${BRANCH}/${VERSION}/lib.zip"
PART_PREFIX="lib-part"
INDEX=1

echo "🌐 [rainbow-release-app]레포에서 ${PART_PREFIX}-*.tar.gz 파일 다운로드 및 압축 해제 시작..."

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# LIB_DIR="${SCRIPT_DIR}/lib"
LIB_DIR="${SCRIPT_DIR}"
DEST="${LIB_DIR}/lib.zip"

if [[ -d "$LIB_DIR" ]]; then
  rm -rf "$LIB_DIR"
fi
mkdir -p "$LIB_DIR"

if curl -fLo "$DEST" "$S3_BASE_URL" 2>/dev/null; then
  echo "🗃️ S3에서 다운로드 성공, 압축 해제: lib.zip → $LIB_DIR"
  unzip -q "$DEST" -d "$LIB_DIR"
  rm -rf "$DEST"
  echo "✅ S3에서 다운로드 및 압축 해제 완료"
else
  while :; do
    FILENAME="${PART_PREFIX}-${INDEX}.zip"
    URL="${GIT_BASE_URL}/lib/${FILENAME}"
    DEST="${LIB_DIR}/${FILENAME}"

    echo "🔎 다운로드 시도: $URL"
    curl -fLo "$DEST" "$URL" 2>/dev/null || break

    echo "🗃️ 압축 해제: $FILENAME → $LIB_DIR"
    unzip -q "$DEST" -d "$LIB_DIR"

    rm -rf "$DEST"

    ((INDEX++))
  done

  echo "✅ GitHub에서 다운로드 및 압축 해제 완료"
fi
