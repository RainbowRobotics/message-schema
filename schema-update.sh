#!/usr/bin/env bash
set -euo pipefail

print_string() {
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

SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

while [ $# -gt 0 ]; do
  case "$1" in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    *) print_string "error" "Unknown option: $1"; exit 1 ;;
  esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
cd "$MAIN_REPO"

if [ ! -d "$SCHEMA_DIR" ]; then
  print_string "error" "Not found: $SCHEMA_DIR"
  exit 1
fi

if [ "$SCRIPT_DIR" != "$MAIN_REPO/$SCHEMA_DIR" ]; then
  print_string "error" "$SCRIPT_NAME must be located in $SCHEMA_DIR"
  exit 1
fi

git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  print_string "error" "Remote not found: $REMOTE_NAME"
  exit 1
}

if [ -f "$(git rev-parse --git-dir)/MERGE_HEAD" ]; then
  print_string "error" "Merge in progress. Resolve/abort merge first."
  exit 1
fi

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
SCHEMA_STASH_HASH=""
TMP_WORKTREE=""
TMP_COMMIT=""
TMP_BASE_DIR="$MAIN_REPO/.git/schema-update-tmp"

print_string "info" "=== STEP 1: Pull $REMOTE_NAME/main into $SCHEMA_DIR ==="

# SCHEMA_DIR 이외 워킹트리가 깨끗해야 실행.
if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
  print_string "error" "SCHEMA_DIR 이외 영역에 staged 변경사항이 있습니다."
  print_string "info" "SCHEMA_DIR 이외에 수정하신 내용은 먼저 커밋/푸시(또는 별도 정리) 후 schema-update를 다시 실행해 주세요."
  exit 1
fi

if [ -n "$(git status --porcelain=v1 -uall -- . ":!$SCHEMA_DIR")" ]; then
  print_string "error" "SCHEMA_DIR 이외 영역에 수정사항이 있습니다."
  print_string "info" "SCHEMA_DIR 이외에 수정하신 내용은 먼저 커밋/푸시(또는 별도 정리) 후 schema-update를 다시 실행해 주세요."
  exit 1
fi

# SCHEMA_DIR 내부 변경은 먼저 로컬 커밋해서 subtree pull 가능 상태로 만든다.
if [ -n "$(git status --porcelain=v1 -uall -- "$SCHEMA_DIR")" ]; then
  print_string "info" "$SCHEMA_DIR 로컬 변경을 먼저 커밋합니다."
  git add -A -- "$SCHEMA_DIR"
  git commit -m "WIP: local $SCHEMA_DIR changes before sync"
fi

print_string "info" "Fetching $REMOTE_NAME/main..."
git fetch "$REMOTE_NAME" main

# 현재 워킹트리에서 pull
if ! GIT_EDITOR=true git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash -m "Merge $REMOTE_NAME/main into $SCHEMA_DIR"; then
  print_string "error" "subtree pull failed (conflict)"
  echo ""
  print_string "warning" "=== Conflict files ==="
  git diff --name-only --diff-filter=U -- "$SCHEMA_DIR" || true
  echo ""
  print_string "warning" "=== Git status (short) ==="
  git status --short -- "$SCHEMA_DIR" || true
  echo ""
  print_string "info" "Resolve conflicts in IDE (conflict markers are in current workspace), then:"
  echo "  1) git add $SCHEMA_DIR"
  echo "  2) git commit"
  echo "  3) git push origin $CURRENT_BRANCH"
  echo "  4) make schema-update"
  exit 1
fi

print_string "success" "Pulled successfully"

print_string "info" "=== STEP 2: Commit/push main repo ($SCHEMA_DIR only) ==="

if [ -n "$(git status --porcelain=v1 -uall -- "$SCHEMA_DIR")" ]; then
  echo ""
  git status --short -uall -- "$SCHEMA_DIR"
  echo ""
  git add -A -- "$SCHEMA_DIR"
  git commit -m "Update $SCHEMA_DIR from $REMOTE_NAME/main"
fi

# pull로 생성된 커밋(또는 위 commit)이 있으면 메인 레포 푸시
if [ "$(git rev-parse HEAD)" != "$(git rev-parse "origin/$CURRENT_BRANCH" 2>/dev/null || echo "")" ]; then
  git push origin "$CURRENT_BRANCH"
  print_string "success" "Pushed to origin/$CURRENT_BRANCH"
else
  print_string "info" "No new commit to push on $CURRENT_BRANCH"
fi

print_string "info" "=== STEP 3: Publish $REMOTE_NAME schema branch ==="

BR="schema/from-$(git config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
  print_string "error" "Cannot determine schema branch from git user.email"
  exit 1
fi

LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR")"
MAIN_COMMIT="$(git rev-parse --short HEAD)"

git fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

if git show-ref --verify --quiet "refs/remotes/$REMOTE_NAME/$BR"; then
  REMOTE_HEAD="$(git rev-parse "$REMOTE_NAME/$BR")"
  REMOTE_TREE="$(git rev-parse "$REMOTE_NAME/$BR^{tree}")"
  if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
    print_string "info" "No schema branch updates"
  else
    NEW_COMMIT="$(printf "Update schemas from main @ %s\n" "$MAIN_COMMIT" | git commit-tree "$LOCAL_TREE" -p "$REMOTE_HEAD")"
    git push "$REMOTE_NAME" "$NEW_COMMIT:refs/heads/$BR"
    print_string "success" "Updated $REMOTE_NAME/$BR"
  fi
else
  TMP="$BR-tmp"
  git branch -D "$TMP" 2>/dev/null || true
  git subtree split --prefix="$SCHEMA_DIR" -b "$TMP" >/dev/null
  git push "$REMOTE_NAME" "$TMP:refs/heads/$BR"
  git branch -D "$TMP" >/dev/null
  print_string "success" "Created $REMOTE_NAME/$BR"
fi

print_string "success" "Done"