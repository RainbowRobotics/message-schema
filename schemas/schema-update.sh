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

stash_schema_if_changed() {
  local before after
  before="$(git rev-parse -q --verify refs/stash 2>/dev/null || true)"
  git stash push -u -m "schema-update-schema-stash-$(date +%s)" -- "$SCHEMA_DIR" >/dev/null 2>&1 || true
  after="$(git rev-parse -q --verify refs/stash 2>/dev/null || true)"
  if [ -n "$after" ] && [ "$before" != "$after" ]; then
    echo "$after"
  else
    echo ""
  fi
}

stash_ref_from_hash() {
  local hash="$1"
  git stash list --format='%gd %H' | awk -v h="$hash" '$2==h{print $1; exit}'
}

restore_schema_stash() {
  local hash="$1"
  [ -z "$hash" ] && return 0
  local ref
  ref="$(stash_ref_from_hash "$hash")"
  [ -z "$ref" ] && return 0
  print_string "info" "Restoring schema changes from $ref..."
  if ! git stash pop "$ref"; then
    print_string "error" "Failed to restore schema stash ($ref)"
    print_string "warning" "Resolve conflicts manually, then rerun schema-update"
    return 1
  fi
  return 0
}

cleanup() {
  if [ -n "$TMP_WORKTREE" ]; then
    git worktree remove --force "$TMP_WORKTREE" >/dev/null 2>&1 || true
    rm -rf "$TMP_WORKTREE" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

cleanup_stale_worktrees() {
  mkdir -p "$TMP_BASE_DIR"
  git worktree prune >/dev/null 2>&1 || true

  # 이전 실행에서 남은 schema-update 임시 worktree 자동 정리
  for d in "$TMP_BASE_DIR"/wt.*; do
    [ -e "$d" ] || continue
    git worktree remove --force "$d" >/dev/null 2>&1 || true
    rm -rf "$d" >/dev/null 2>&1 || true
  done

  git worktree prune >/dev/null 2>&1 || true
}

print_string "info" "=== STEP 1: Sync $SCHEMA_DIR from $REMOTE_NAME/main ==="
cleanup_stale_worktrees

# SCHEMA_DIR 밖 변경은 그대로 두고 SCHEMA_DIR만 잠깐 비워서 동기화.
SCHEMA_STASH_HASH="$(stash_schema_if_changed)"
if [ -n "$SCHEMA_STASH_HASH" ]; then
  print_string "info" "Stashed local $SCHEMA_DIR changes"
fi

TMP_WORKTREE="$(mktemp -d "$TMP_BASE_DIR/wt.XXXXXX")"
git worktree add --detach "$TMP_WORKTREE" HEAD >/dev/null

if ! (
  cd "$TMP_WORKTREE"
  git fetch "$REMOTE_NAME" main >/dev/null
  GIT_EDITOR=true git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash -m "Merge $REMOTE_NAME/main into $SCHEMA_DIR" >/dev/null
); then
  print_string "error" "subtree pull failed (conflict)"
  echo ""
  print_string "warning" "=== Conflict files ==="
  git -C "$TMP_WORKTREE" diff --name-only --diff-filter=U -- "$SCHEMA_DIR" || true
  echo ""
  print_string "warning" "=== Git status (short) ==="
  git -C "$TMP_WORKTREE" status --short -- "$SCHEMA_DIR" || true
  echo ""
  print_string "info" "This conflict occurred in a temporary worktree."
  print_string "info" "Apply equivalent conflict resolution in your main workspace, then:"
  echo "  1) edit conflicted files under $SCHEMA_DIR"
  echo "  2) git add $SCHEMA_DIR"
  echo "  3) git commit"
  echo "  4) git push origin $CURRENT_BRANCH"
  echo "  5) make schema-update"
  echo ""
  print_string "info" "Tip: open the first conflicted file in your workspace:"
  echo "  code -g \$(git -C \"$TMP_WORKTREE\" diff --name-only --diff-filter=U -- \"$SCHEMA_DIR\" | head -n 1)"
  print_string "warning" "Resolve conflict on your branch first, commit, then rerun"
  restore_schema_stash "$SCHEMA_STASH_HASH" || true
  exit 1
fi

TMP_COMMIT="$(git -C "$TMP_WORKTREE" rev-parse HEAD)"

# 현재 브랜치 워킹트리에 SCHEMA_DIR만 반영
git restore --source="$TMP_COMMIT" --staged --worktree -- "$SCHEMA_DIR"

# 사용자 로컬 schema 변경 복원
if ! restore_schema_stash "$SCHEMA_STASH_HASH"; then
  exit 1
fi
SCHEMA_STASH_HASH=""

print_string "success" "Schema sync applied"

print_string "info" "=== STEP 2: Commit/push main repo ($SCHEMA_DIR only) ==="
if [ -n "$(git status --porcelain=v1 -uall -- "$SCHEMA_DIR")" ]; then
  echo ""
  git status --short -uall -- "$SCHEMA_DIR"
  echo ""
  git add -A -- "$SCHEMA_DIR"
  git commit -m "Update $SCHEMA_DIR from $REMOTE_NAME/main"
  git push origin "$CURRENT_BRANCH"
  print_string "success" "Pushed to origin/$CURRENT_BRANCH"
else
  print_string "info" "No $SCHEMA_DIR changes to commit"
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
