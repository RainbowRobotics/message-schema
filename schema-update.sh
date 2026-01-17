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

# 개인 작업용 schema 브랜치
BR="schema/from-$(git -C "$MAIN_REPO" config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
  echo "Error: Cannot determine branch name. Check git user.email"
  exit 1
fi
echo "schema branch => $BR"

# 현재 메인 레포의 schema 상태
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)

echo "local  tree => $LOCAL_TREE"
echo "main commit => $MAIN_COMMIT"

# remote 존재 여부 확인
git -C "$MAIN_REPO" remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  echo "Error: Remote '$REMOTE_NAME' not found in main repo"
  exit 1
}

# 원격 개인 브랜치 fetch (없을 수도 있음)
git -C "$MAIN_REPO" fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

# ===== 기존 브랜치가 있는 경우 =====
if git -C "$MAIN_REPO" show-ref --verify --quiet "$REMOTE_REF"; then
  REMOTE_TREE=$(git -C "$MAIN_REPO" rev-parse "$REMOTE_NAME/$BR^{tree}")
  echo "remote tree => $REMOTE_TREE"

    # schema 내용이 같으면 스킵
  if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
    echo "No diff. Skip."
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
  echo "Creating new branch: $BR"
  TMP="$BR-tmp"

  # schema 디렉토리만 분리해서 브랜치 생성
  git -C "$MAIN_REPO" subtree split --prefix="$SCHEMA_DIR" -b "$TMP"
  git -C "$MAIN_REPO" push "$REMOTE_NAME" "$TMP:refs/heads/$BR"
  git -C "$MAIN_REPO" branch -D "$TMP"
fi

echo "Pushed to message-schema: $BR"
