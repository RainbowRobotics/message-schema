#!/bin/bash
set -euo pipefail

# 기본값 (필요하면 옵션으로 덮어씀)
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 옵션 처리
while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    *) echo "알 수 없는 옵션: $1"; exit 1 ;;
  esac
done

# 항상 메인 레포 루트 기준으로 동작
MAIN_REPO="$(git rev-parse --show-toplevel)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 현재 서브트리 환경에서 실행되는지 확인
if [ -d "$SCRIPT_DIR/.git" ]; then
  echo "Error: 이 스크립트는 부모 레포지토리(서브트리 컨텍스트)에서 실행해야 합니다."
  echo "message-schema 레포지토리 내에서 직접 실행하지 마세요."
  exit 1
fi


# 개인 작업용 schema 브랜치
BR="schema/from-$(git -C "$MAIN_REPO" config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
  echo "Error: 브랜치 이름을 결정할 수 없습니다. git user.email를 확인하세요."
  exit 1
fi
echo "schema branch => $BR"

# 현재 메인 레포의 schema 상태
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)

echo "로컬 트리 => $LOCAL_TREE"
echo "메인 커밋 => $MAIN_COMMIT"

# remote 존재 여부 확인
git -C "$MAIN_REPO" remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  echo "Error: '$REMOTE_NAME' 원격 레포지토리를 찾을 수 없습니다."
  exit 1
}

# 원격 개인 브랜치 fetch (없을 수도 있음)
git -C "$MAIN_REPO" fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

# ===== 기존 브랜치가 있는 경우 =====
if git -C "$MAIN_REPO" show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_TREE=$(git -C "$MAIN_REPO" rev-parse "$REMOTE_NAME/$BR^{tree}")
    echo "원격 트리 => $REMOTE_TREE"

        # schema 내용이 같으면 스킵
    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        echo "변경이 없습니다. 스킵합니다."
        exit 0
    fi

    # 원격 브랜치 기준 worktree에서 schema만 갱신
    WORK_DIR=$(mktemp -d)
    trap "git -C '$MAIN_REPO' worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT

    git -C "$MAIN_REPO" worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"

    (cd "$WORK_DIR" && \
        rm -rf -- ./* ./.??* 2>/dev/null || true && \
        git read-tree --reset -u "$LOCAL_TREE" && \
        git add -A && \
        git commit -m "Update schemas from main repo @ $MAIN_COMMIT" && \
        git push "$REMOTE_NAME" "HEAD:refs/heads/$BR")

else
    # 기존 브랜치가 없으면 새로 생성
    echo " 새로운 브랜치 생성: $BR"
    TMP="$BR-tmp"

    # schema 디렉토리만 분리해서 브랜치 생성
    git -C "$MAIN_REPO" subtree split --prefix="$SCHEMA_DIR" -b "$TMP"
    git -C "$MAIN_REPO" push "$REMOTE_NAME" "$TMP:refs/heads/$BR"
    git -C "$MAIN_REPO" branch -D "$TMP"
fi

echo "성공: message-schema에 푸시됨: $BR"
