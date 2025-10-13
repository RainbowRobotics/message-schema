#!/bin/bash
set -e

BRANCH=main
VERSION=version
SERVICE_NAME=service-name
S3_BASE_URL="https://rainbow-deploy.s3.ap-northeast-2.amazonaws.com/robot-repeater-server/${BRANCH}/${SERVICE_NAME}/${VERSION}.zip"
PART_PREFIX="build"
INDEX=1

echo "🌐 바이너리 파일 다운로드 및 압축 해제 시작"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/${SERVICE_NAME}"
DEST="${LIB_DIR}/${VERSION}.zip"

mkdir -p "$LIB_DIR"

if curl -fLo "$DEST" "$S3_BASE_URL" 2>/dev/null; then
  echo "🗃️ S3에서 다운로드 성공, 압축 해제 진행중..."
  unzip -oq "$DEST" -d "$LIB_DIR"
  rm -rf "$DEST"
  echo "✅ S3에서 다운로드 및 압축 해제 완료"
else
  echo "❌ S3에서 다운로드 실패, 찾을 수 없는 버전입니다."
  exit 1
fi