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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 현재 서브트리 환경에서 실행되는지 확인
if [ -d "$SCRIPT_DIR/.git" ]; then
    echo "Error: 이 스크립트는 부모 레포지토리(서브트리 컨텍스트)에서 실행해야 합니다."
    echo "message-schema 레포지토리 내에서 직접 실행하지 마세요."
    exit 1
fi

# remote 없으면 추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

# stash 생성 여부 확인
STASH_CREATED=0

STASH_OID=""

# 작업 트리가 clean하지 않으면 stash 생성
if [[ -n "$(git status --porcelain)" ]]; then
    before_cnt="$(git stash list | wc -l | tr -d ' ')"

    # stash 생성
    git stash push -u -m "schema-sync auto-stash" >/dev/null 2>&1 || true
    after_cnt="$(git stash list | wc -l | tr -d ' ')"

    # stash 생성 여부 확인
    if (( after_cnt > before_cnt )); then
        STASH_CREATED=1

        # 지금 시점의 stash top(방금 만든 stash)을 해시로 고정
        STASH_OID="$(git rev-parse -q --verify stash@{0} 2>/dev/null || true)"

        if [[ -z "$STASH_OID" ]]; then
        echo "Error:  stash 생성은 성공했지만, 스태시 OID를 읽을 수 없습니다."
        exit 1
        fi
    fi

    if [[ -n "$(git status --porcelain)" ]]; then
        echo "작업 트리가 clean하지 않습니다. git subtree pull를 실행할 수 없습니다."
        git status -sb
        exit 1
    fi
fi

# remote 브랜치 fetch
git fetch "$REMOTE_NAME" main

# schema 동기화
SUBTREE_OUT="$(git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main" 2>&1 | tee /dev/stderr)"

# 변경이 없으면(이미 최신) → 정상 종료
if echo "$SUBTREE_OUT" | grep -q "Subtree is already at commit"; then
    if [[ -n "$STASH_REF" ]]; then
        if git stash apply "$STASH_REF" >/dev/null; then
            git stash drop "$STASH_REF" >/dev/null
        fi
    fi
    exit 0
fi

# stash 만든게 있으면 적용

if (( STASH_CREATED == 1 )); then
    # stash@{n} 중에서 방금 저장한 OID와 매칭되는 엔트리를 찾는다
    STASH_REF="$(git stash list --format='%gd %H' | awk -v oid="$STASH_OID" '$2==oid {print $1; exit}')"

    if [[ -z "$STASH_REF" ]]; then
        echo "Error: cannot find stash entry for $STASH_OID"
        echo "현재 stash 목록:"
        git stash list | head -n 5
        exit 1
    fi

    if git stash apply "$STASH_REF" >/dev/null; then
        git stash drop "$STASH_REF" >/dev/null
    else
        echo "stash 적용 중 충돌이 발생했습니다. 'message-schema' 레포지토리에서 충돌을 해결한 후!! 메인 레포지토리에서 아래 명령어를 실행하세요."
        echo "  git stash drop $STASH_REF && make schema-sync"
        exit 1
    fi
fi