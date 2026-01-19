#!/bin/bash
set -euo pipefail

# 기본값
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

# 메인 레포 루트
MAIN_REPO="$(git rev-parse --show-toplevel)"

# SCHEMA_DIR 디렉토리 확인
if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
  echo "Error: '$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
  exit 1
fi

# SCHEMA_DIR 디렉토리에 변경사항이 있으면 자동 커밋
cd "$MAIN_REPO"
if ! git diff --quiet HEAD -- "$SCHEMA_DIR" || ! git diff --cached --quiet -- "$SCHEMA_DIR"; then
  echo "schemas 디렉토리에 변경사항 감지. 메인 레포에 커밋합니다..."
  git add "$SCHEMA_DIR"
  git commit -m "Update schemas"

  # 현재 브랜치 이름 가져오기
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  echo "메인 레포 푸시: $CURRENT_BRANCH"
  git push origin "$CURRENT_BRANCH"
fi


# 개인 브랜치명 생성
BR="schema/from-$(git -C "$MAIN_REPO" config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
  echo "Error: 브랜치 이름을 결정할 수 없습니다."
  exit 1
fi
echo "schema branch => $BR"

# 현재 상태
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)

echo "local  tree => $LOCAL_TREE"
echo "main commit => $MAIN_COMMIT"

# remote 확인
git -C "$MAIN_REPO" remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  echo "Error: '$REMOTE_NAME' 원격 레포지토리를 찾을 수 없습니다."
  echo "message-schema 레포지토리에서 README.md에 초기 1회 세팅을 참고하세요."
  exit 1
}

# 원격 브랜치 fetch
git -C "$MAIN_REPO" fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

# 기존 브랜치가 있는 경우
if git -C "$MAIN_REPO" show-ref --verify --quiet "$REMOTE_REF"; then
  REMOTE_TREE=$(git -C "$MAIN_REPO" rev-parse "$REMOTE_NAME/$BR^{tree}")
  echo "remote tree => $REMOTE_TREE"

  if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
    echo "변경사항 없음."
    exit 0
  fi

  WORK_DIR=$(mktemp -d)
  trap "git -C '$MAIN_REPO' worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT

  git -C "$MAIN_REPO" worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"

  (cd "$WORK_DIR" && \
    find . -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} + && \
    git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x && \
    git add -A && \
    git commit -m "Update schemas from main repo @ $MAIN_COMMIT" && \
    git push "$REMOTE_NAME" "HEAD:refs/heads/$BR")

else
  echo "새 브랜치 생성: $BR"
  TMP="$BR-tmp"

  git -C "$MAIN_REPO" branch -D "$TMP" 2>/dev/null || true
  git -C "$MAIN_REPO" subtree split --prefix="$SCHEMA_DIR" -b "$TMP"
  git -C "$MAIN_REPO" push "$REMOTE_NAME" "$TMP:refs/heads/$BR"
  git -C "$MAIN_REPO" branch -D "$TMP"
fi

echo "성공: $REMOTE_NAME/$BR 푸시 완료"
