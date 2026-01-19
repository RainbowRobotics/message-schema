#!/usr/bin/env bash
set -euo pipefail

MAIN_REPO="$(git rev-parse --show-toplevel)"

# 기본값
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 서브트리로 포함되었는지 확인
if [ -d "$SCRIPT_DIR/.git" ]; then
    echo "Error: 이 스크립트는 서브트리 컨텍스트에서 실행해야 합니다."
    exit 1
fi

cd "$MAIN_REPO"

# SCHEMA_DIR 존재 여부 확인
if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
    echo "Error: '$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
    exit 1
fi

# remote 확인/추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
    git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

STASH_REF=""

# EXIT 시 stash 원복
restore_stash() {
    if [[ -n "$STASH_REF" ]]; then
        if git stash apply "$STASH_REF" >/dev/null 2>&1; then
            git stash drop "$STASH_REF" >/dev/null 2>&1 || true
            echo "작업 디렉토리가 복원되었습니다."
        else
            echo "Warning: stash 복원 실패. 수동 처리 필요:"
            echo "  git stash apply $STASH_REF"
            echo "  git stash drop $STASH_REF"
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
        echo "Error: stash가 생성되지 않았습니다"
        exit 1
    fi

    echo "작업 중인 변경사항을 임시 저장: $STASH_REF"

    # stash 후에도 더러우면 실패
    if [[ -n "$(git status --porcelain)" ]]; then
        echo "Error: stash 후에도 작업 디렉토리가 깨끗하지 않습니다"
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

echo "$SUBTREE_OUT"

# "이미 최신" 메시지는 성공으로 처리
if echo "$SUBTREE_OUT" | grep -q "Subtree is already at commit"; then
    echo "schemas는 이미 최신 상태입니다."
    exit 0
fi

if [ $RC -ne 0 ]; then
    echo "Error: subtree pull 실패 (exit code: $RC)"
    exit $RC
fi

echo "schemas 동기화 완료"
