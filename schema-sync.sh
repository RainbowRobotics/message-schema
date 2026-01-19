#!/usr/bin/env bash
set -euo pipefail

# 컬러 출력 함수
function print_string(){
  local RED='\033[0;31m'
  local GREEN='\033[0;32m'
  local YELLOW='\033[1;33m'
  local BLUE='\033[0;34m'
  local NC='\033[0m'

  case "$1" in
    "error") echo -e "${RED}${2}${NC}" ;;
    "success") echo -e "${GREEN}${2}${NC}" ;;
    "warning") echo -e "${YELLOW}${2}${NC}" ;;
    "info") echo -e "${BLUE}${2}${NC}" ;;
  esac
}

# 기본값
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 옵션 처리
while [[ $# -gt 0 ]]; do
    case $1 in
        --dir) SCHEMA_DIR="$2"; shift 2 ;;
        --remote) REMOTE_NAME="$2"; shift 2 ;;
        *) print_string "error" "Unknown option: $1"; exit 1 ;;
    esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 서브트리로 포함되었는지 확인 (스크립트가 메인 레포 내부에 있어야 함)
if [ "$MAIN_REPO" != "$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo '')" ]; then
    print_string "error" "이 스크립트는 서브트리로 포함된 상태에서만 실행해야 합니다."
    print_string "info" "message-schema 레포지토리를 독립적으로 클론해서 실행하지 마세요."
    exit 1
fi

# SCHEMA_DIR 존재 여부 확인
if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
    print_string "error" "'$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
    exit 1
fi

cd "$MAIN_REPO"

# remote 확인/추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
    git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

STASH_REF=""

# EXIT 시 stash 원복
restore_stash() {
    if [[ -n "$STASH_REF" ]]; then
        if git stash apply "$STASH_REF" >/dev/null 2>&1; then
            git stash drop "$STASH_REF" >/dev/null 2>&1 || true
            print_string "info" "작업 디렉토리가 복원되었습니다."
        else
            print_string "warning" "stash 복원 실패. 수동 처리 필요:"
            print_string "info" "  git stash apply $STASH_REF"
            print_string "info" "  git stash drop $STASH_REF"
        fi
    fi
}

trap restore_stash EXIT

# 작업 디렉토리가 더러우면 stash
if [[ -n "$(git status --porcelain)" ]]; then
    TOKEN="$(date +%s)-$$"
    MSG="schema-sync auto-stash ${TOKEN}"

    git stash push -u -m "$MSG"

    # set -e 영향 받지 않도록
    set +e
    STASH_REF="$(git stash list --format='%gd %s' | grep -F "$MSG" | head -1 | cut -d' ' -f1)"
    set -e

    if [ -z "$STASH_REF" ]; then
        print_string "error" "stash가 생성되지 않았습니다"
        exit 1
    fi

    echo "작업 중인 변경사항을 임시 저장: $STASH_REF"

    # stash 후에도 더러우면 실패
    if [[ -n "$(git status --porcelain)" ]]; then
        print_string "error" "stash 후에도 작업 디렉토리가 깨끗하지 않습니다"
        exit 1
    fi
fi

# subtree pull 실행
git fetch "$REMOTE_NAME" main

set +e
SUBTREE_OUT="$(git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main" 2>&1)"
RC=$?
set -e

print_string "info" "$SUBTREE_OUT"

# "이미 최신" 메시지는 성공으로 처리
if echo "$SUBTREE_OUT" | grep -q "Subtree is already at commit"; then
    print_string "info" "schemas는 이미 최신 상태입니다."
    exit 0
fi

if [ $RC -ne 0 ]; then
    print_string "error" "subtree pull 실패 (exit code: $RC)"
    exit $RC
fi

print_string "success" "schemas 동기화 완료"
