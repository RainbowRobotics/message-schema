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

# subtree는 워킹트리가 더러우면 막힘 → 필요 시 내 stash만 만들어 치움 (충돌 방지)
STASH_REF=""

dirty=0
git diff --quiet || dirty=1
git diff --cached --quiet || dirty=1
if [[ -n "$(git ls-files --others --exclude-standard)" ]]; then dirty=1; fi

if [[ "$dirty" -eq 1 ]]; then
  before_top="$(git stash list -n 1 | sed -n 's/^\(stash@{[0-9]\+}\).*/\1/p')"
  git stash push -u -m "schema-sync auto-stash" >/dev/null
  after_top="$(git stash list -n 1 | sed -n 's/^\(stash@{[0-9]\+}\).*/\1/p')"

  # 방금 만든 stash ref를 확실히 잡는다 (pop 금지)
  if [[ -n "$after_top" && "$after_top" != "$before_top" ]]; then
    STASH_REF="$after_top"
  else
    echo "Error: failed to identify created stash entry"
    exit 1
  fi
fi

# remote 브랜치 fetch
git fetch "$REMOTE_NAME" main

# schema 동기화
git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main"

# 내가 만든 stash만 복원 (apply 성공 시에만 drop)
if [[ -n "$STASH_REF" ]]; then
  if git stash apply "$STASH_REF" >/dev/null; then
    git stash drop "$STASH_REF" >/dev/null
  else
    echo "Conflict while applying $STASH_REF. Resolve conflicts, then drop manually:"
    echo "  git stash drop $STASH_REF"
    exit 1
  fi
fi