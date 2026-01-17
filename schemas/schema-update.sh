#!/bin/bash
set -euo pipefail

# 기본값 설정
MAIN_REPO="."
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 인자 파싱
while [[ $# -gt 0 ]]; do
    case $1 in
        --dir)
            SCHEMA_DIR="$2"
            shift 2
            ;;
        --origin)
            REMOTE_NAME="$2"
            shift 2
            ;;
        *)
            MAIN_REPO="$1"
            shift
            ;;
    esac
done

cd "$(dirname "$0")"

# 사용자 이메일 기반 브랜치명 생성
BR="schema/from-$(git config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
    echo "Error: Cannot determine branch name. Check git user.email"
    exit 1
fi
echo "schema branch => $BR"

# 메인 레포의 schemas 디렉토리 트리 해시 계산
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)
echo "local  tree => $LOCAL_TREE"
echo "main commit => $MAIN_COMMIT"

# origin에서 최신 정보 가져오기
git fetch origin || true

# 원격에 브랜치가 이미 존재하는지 확인
if git show-ref --verify --quiet "refs/remotes/origin/$BR"; then
    REMOTE_TREE=$(git rev-parse "origin/$BR:")
    echo "remote tree => $REMOTE_TREE"
    
    # 변경사항이 없으면 스킵
    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        echo "No diff. Skip."
        exit 0
    fi
    
    # 기존 브랜치 기반으로 체크아웃하고 내용 교체
    git checkout -B "$BR" "origin/$BR"
    rm -rf *
    git -C "$MAIN_REPO" archive "HEAD:$SCHEMA_DIR" | tar -x
    git add -A
    git commit -m "Update schemas from main repo @ $MAIN_COMMIT"
    git push origin "$BR"
else
    # 브랜치가 없으면 subtree split으로 새로 생성
    echo "Creating new branch: $BR"
    TMP="$BR-tmp"
    (cd "$MAIN_REPO" && \
        git branch -D "$TMP" 2>/dev/null || true && \
        git subtree split --prefix="$SCHEMA_DIR" -b "$TMP" && \
        git push "$REMOTE_NAME" "$TMP:refs/heads/$BR" && \
        git branch -D "$TMP")
fi

echo "Pushed to message-schema: $BR"