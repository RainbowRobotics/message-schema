#!/usr/bin/env bash
set -euo pipefail

# 컬러 출력 함수
function print_string() {
  local RED=$'\033[0;31m'
  local GREEN=$'\033[0;32m'
  local YELLOW=$'\033[1;33m'
  local BLUE=$'\033[0;34m'
  local NC=$'\033[0m'

  case "$1" in
    error)   printf "%s%s%s\n" "$RED"   "$2" "$NC" ;;
    success) printf "%s%s%s\n" "$GREEN" "$2" "$NC" ;;
    warning) printf "%s%s%s\n" "$YELLOW""$2" "$NC" ;;
    info)    printf "%s%s%s\n" "$BLUE"  "$2" "$NC" ;;
    *)       printf "%s\n" "$2" ;;
  esac
}

# y/n 확인 유틸 (function 키워드 없이)
function confirm_or_exit() {
  local message="$1"
  echo ""
  print_string "warning" "${message}"
  echo ""
  read -r -p "계속 진행하시겠습니까? (y/N): " ANS

  ANS="$(printf '%s' "$ANS" | tr -d '\r' | sed 's/^ *//; s/ *$//' | tr '[:upper:]' '[:lower:]')"

  case "$ANS" in
    y|yes) return 0 ;;
    *) print_string "info" "취소되었습니다."; exit 0 ;;
  esac
}

# 기본값
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"
ASSUME_YES=0

# 옵션 처리
while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    --yes) ASSUME_YES=1; shift 1 ;;
    *) print_string "error" "Unknown option: $1"; exit 1 ;;
  esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 서브트리 컨텍스트 확인 (독립 클론 방지)
if [ -d "$SCRIPT_DIR/.git" ]; then
  print_string "error" "이 스크립트는 서브트리 컨텍스트(메인 레포)에서 실행해야 합니다."
  exit 1
fi

cd "$MAIN_REPO"

# SCHEMA_DIR 존재 확인
if [ ! -d "$SCHEMA_DIR" ]; then
  print_string "error" "'$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
  exit 1
fi

# remote 확인/추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

print_string "info" "fetch: $REMOTE_NAME/main"
git fetch "$REMOTE_NAME" main

# 로컬 schemas 트리(HEAD 기준) 존재 확인
LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || true)"
if [ -z "${LOCAL_TREE:-}" ]; then
  print_string "error" "HEAD에 '$SCHEMA_DIR' 트리가 없습니다. subtree 초기 상태/경로를 확인하세요."
  exit 1
fi

REMOTE_TREE="$(git rev-parse "$REMOTE_NAME/main^{tree}")"

print_string "info" "local  tree (HEAD:$SCHEMA_DIR) => $LOCAL_TREE"
print_string "info" "remote tree ($REMOTE_NAME/main) => $REMOTE_TREE"

# schemas의 로컬 변경(추가/수정/staged/untracked) 감지
HAS_CHANGES=0
if ! git diff --quiet -- "$SCHEMA_DIR"; then HAS_CHANGES=1; fi
if ! git diff --cached --quiet -- "$SCHEMA_DIR"; then HAS_CHANGES=1; fi
if [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then HAS_CHANGES=1; fi

# HEAD 기준 schemas 트리가 원격 main과 다른지(커밋돼서 clean이어도 잡힘)
TREE_DIFF=0
if [ "$LOCAL_TREE" != "$REMOTE_TREE" ]; then
  TREE_DIFF=1
fi

# 이미 동일 + 로컬 변경 없음이면 종료
if [ "$TREE_DIFF" -eq 0 ] && [ "$HAS_CHANGES" -eq 0 ]; then
  print_string "success" "schemas는 이미 $REMOTE_NAME/main과 동일합니다."
  exit 0
fi

if [ "$HAS_CHANGES" -eq 1 ]; then
  confirm_or_exit \
"'$SCHEMA_DIR' 아래에 로컬 변경이 있습니다. 동기화하면 '$SCHEMA_DIR'의 로컬 변경(수정/추가/untracked/staged)이 삭제됩니다."
else
  # 로컬 작업은 없지만, 커밋된 상태(워킹트리 clean)라도 현재 schemas 상태가 덮일 수 있음
  confirm_or_exit \
"'$SCHEMA_DIR'의 현재 커밋 상태가 '$REMOTE_NAME/main'과 다릅니다. 최신 main을 반영하기 위해 동기화합니다.
(참고: 로컬 변경은 없지만, 현재 schemas 상태는 동기화 커밋으로 덮일 수 있습니다.)"
fi

# 파괴적 정리 (schemas만)
git restore -SW -- "$SCHEMA_DIR" 2>/dev/null || { print_string "error" "schemas 초기화(git restore) 실패"; exit 1; }
git clean -fd -- "$SCHEMA_DIR" || { print_string "error" "schemas 정리(git clean) 실패"; exit 1; }

# subtree pull
print_string "info" "subtree pull: $REMOTE_NAME/main -> $SCHEMA_DIR"
set +e
SUBTREE_OUT="$(git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main" 2>&1)"
RC=$?
set -e
print_string "info" "$SUBTREE_OUT"

if [ $RC -ne 0 ]; then
  print_string "error" "subtree pull 실패 (exit code: $RC)"
  exit $RC
fi

# 검증: pull 과정에서 원격이 더 최신으로 움직일 수 있으니 재-fetch 후 비교
git fetch "$REMOTE_NAME" main
REMOTE_TREE_AFTER="$(git rev-parse "$REMOTE_NAME/main^{tree}")"
LOCAL_TREE_AFTER="$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || true)"

print_string "info" "local  tree after => ${LOCAL_TREE_AFTER:-<none>}"
print_string "info" "remote tree after => $REMOTE_TREE_AFTER"

if [ -z "$LOCAL_TREE_AFTER" ] || [ "$LOCAL_TREE_AFTER" != "$REMOTE_TREE_AFTER" ]; then
  print_string "error" "동기화 후에도 '$SCHEMA_DIR'이(가) '$REMOTE_NAME/main'과 동일하지 않습니다."
  exit 1
fi

print_string "success" "schemas 동기화 완료 (message-schema/main과 동일)"
