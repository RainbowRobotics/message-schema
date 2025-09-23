#!/bin/bash
set -e

BRANCH=main
VERSION=version
GIT_BASE_URL="https://github.com/rainbow-mobile/rainbow-release-apps/raw/refs/heads/${BRANCH}/slamnav2/${VERSION}"
S3_BASE_URL="https://rainbow-deploy.s3.ap-northeast-2.amazonaws.com/slamnav2/${BRANCH}/${VERSION}/build.zip"
PART_PREFIX="build"
INDEX=1

echo "🌐 바이너리 파일 다운로드 및 압축 해제 시작"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_DIR="${SCRIPT_DIR}"
DEST="${LIB_DIR}/build.zip"

if curl -fLo "$DEST" "$S3_BASE_URL" 2>/dev/null; then
  echo "🗃️ S3에서 다운로드 성공, 압축 해제 진행중..."
  unzip -oq "$DEST" -d "$LIB_DIR"
  rm -rf "$DEST"
  echo "✅ S3에서 다운로드 및 압축 해제 완료"
else
  echo "❌ S3에서 다운로드 실패, 찾을 수 없는 버전입니다."
  exit 1
fi
