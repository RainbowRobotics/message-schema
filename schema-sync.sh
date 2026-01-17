#!/usr/bin/env bash
set -euo pipefail

# 기본값 (필요하면 옵션으로 덮어씀)
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 옵션 처리
while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

# remote 존재 여부 확인
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || git remote add "$REMOTE_NAME" https://github.com/RainbowRobotics/message-schema

# remote 브랜치 fetch
git fetch "$REMOTE_NAME" main

# schema 동기화
git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash -m "Sync schemas from ${REMOTE_NAME}/main"
