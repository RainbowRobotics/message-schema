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
        *) print_string "error" "알 수 없는 옵션: $1"; exit 1 ;;
    esac
done

# 메인 레포 루트
MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 서브트리로 포함되었는지 확인 (스크립트가 메인 레포 내부에 있어야 함)
if [ "$MAIN_REPO" != "$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo '')" ]; then
    print_string "error" "이 스크립트는 서브트리로 포함된 상태에서만 실행해야 합니다."
    print_string "info" "message-schema 레포지토리를 독립적으로 클론해서 실행하지 마세요."
    exit 1
fi

# SCHEMA_DIR 디렉토리 확인
if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
    print_string "error" "'$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
    exit 1
fi

cd "$MAIN_REPO"

# SCHEMA_DIR 디렉토리에 변경사항이 있으면 자동 커밋
if ! git diff --quiet HEAD -- "$SCHEMA_DIR" \
  || ! git diff --cached --quiet -- "$SCHEMA_DIR" \
  || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "schemas 디렉토리에 변경사항 감지. 메인 레포에 커밋합니다..."

    # schemas만 커밋 (다른 staged 파일 제외)
    git add "$SCHEMA_DIR"

    # 다른 staged 파일이 있으면 경고
    if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
        print_string "warning" "$SCHEMA_DIR 외 다른 staged 파일도 있습니다. 커밋을 취소합니다."
        git reset HEAD "$SCHEMA_DIR"
        exit 1
    fi

    git commit -m "Update schemas"

    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    echo "메인 레포 푸시: $CURRENT_BRANCH"

    # push 실패 시 에러 처리
    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "메인 레포 push 실패. 충돌을 해결하고 다시 시도하세요."
        exit 1
    fi
fi


# 개인 브랜치명 생성
BR="schema/from-$(git -C "$MAIN_REPO" config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
    print_string "error" "브랜치 이름을 결정할 수 없습니다."
    exit 1
fi
echo "schema branch => $BR"

# 현재 상태
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)

print_string "info" "local  tree => $LOCAL_TREE"
print_string "info" "main commit => $MAIN_COMMIT"

# remote 확인
git -C "$MAIN_REPO" remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
    print_string "error" "'$REMOTE_NAME' 원격 레포지토리를 찾을 수 없습니다."
    print_string "info" "message-schema 레포지토리에서 README.md에 초기 1회 세팅을 참고하세요."
    exit 1
}

# 원격 브랜치 fetch
git -C "$MAIN_REPO" fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

# 기존 브랜치가 있는 경우
if git -C "$MAIN_REPO" show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_TREE=$(git -C "$MAIN_REPO" rev-parse "$REMOTE_NAME/$BR^{tree}")
    print_string "info" "remote tree => $REMOTE_TREE"

    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        print_string "info" "변경사항 없음."
        exit 0
    fi

    WORK_DIR=$(mktemp -d)
    trap "git -C '$MAIN_REPO' worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT

    git -C "$MAIN_REPO" worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"

    if !(cd "$WORK_DIR" && \
        find . -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} + && \
        git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x && \
        git add -A && \
        git commit -m "Update schemas from main repo @ $MAIN_COMMIT" && \
        git push "$REMOTE_NAME" "HEAD:refs/heads/$BR"); then
        print_string "error" "message-schema 푸시 실패"
        exit 1
    fi

else
    print_string "info" "새 브랜치 생성: $BR"
    TMP="$BR-tmp"

    git -C "$MAIN_REPO" branch -D "$TMP" 2>/dev/null || true
    git -C "$MAIN_REPO" subtree split --prefix="$SCHEMA_DIR" -b "$TMP"

    if ! git -C "$MAIN_REPO" push "$REMOTE_NAME" "$TMP:refs/heads/$BR"; then
        print_string "error" "message-schema 초기 브랜치 생성 실패"
        git branch -D "$TMP" 2>/dev/null || true
        exit 1
    fi

    git -C "$MAIN_REPO" branch -D "$TMP"
fi

print_string "success" "$REMOTE_NAME/$BR 푸시 완료"
