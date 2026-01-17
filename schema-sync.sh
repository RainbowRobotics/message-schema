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

# 항상 메인 레포 루트 기준으로 동작
MAIN_REPO="$(git rev-parse --show-toplevel)"

# subtree prefix 확인
if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
  echo "Error: Directory '$SCHEMA_DIR' not found in $MAIN_REPO"
  exit 1
fi


# remote 없으면 추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

STASH_CREATED=0

if [[ -n "$(git status --porcelain)" ]]; then
  before_cnt="$(git stash list | wc -l | tr -d ' ')"

  git stash push -u -m "schema-sync auto-stash" >/dev/null 2>&1 || true
  after_cnt="$(git stash list | wc -l | tr -d ' ')"

  if (( after_cnt > before_cnt )); then
    STASH_CREATED=1
  fi

  if [[ -n "$(git status --porcelain)" ]]; then
    echo "Working tree is still dirty; cannot run git subtree pull."
    git status -sb
    exit 1
  fi
fi

git fetch "$REMOTE_NAME" main

git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main"

if (( STASH_CREATED == 1 )); then
  if git stash apply "stash@{0}" >/dev/null; then
    git stash drop "stash@{0}" >/dev/null
  else
    echo "Conflict while applying stash@{0}. Resolve conflicts, then drop manually:"
    echo "  git stash drop stash@{0}"
    exit 1
  fi
fi