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
  print_string "error" "$SCRIPT_NAME must be located at $SCHEMA_DIR/$SCRIPT_NAME"
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
SUCCESS=false
SCHEMA_STASH_HASH=""
OUTSIDE_STASH_HASH=""
LOCAL_SCHEMA_COMMITTED=false
LAST_COMMIT_SHA=""

stash_if_changed() {
  local msg="$1"
  shift
  local before after
  before="$(git rev-parse -q --verify refs/stash 2>/dev/null || true)"
  if [ $# -eq 0 ]; then
    git stash push -u -m "$msg" >/dev/null 2>&1 || true
  else
    git stash push -u -m "$msg" -- "$@" >/dev/null 2>&1 || true
  fi
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

restore_stash() {
  local hash="$1"
  local label="$2"
  [ -z "$hash" ] && return 0

  local ref
  ref="$(stash_ref_from_hash "$hash")"
  [ -z "$ref" ] && return 0

  if [ -f "$(git rev-parse --git-dir)/MERGE_HEAD" ]; then
    print_string "warning" "Merge in progress. Keep $label stash: $ref"
    return 1
  fi

  print_string "info" "Restoring $label from $ref..."
  if ! git stash pop "$ref"; then
    print_string "warning" "Conflict detected while restoring. Trying stash apply..."
    if git stash apply "$ref"; then
      print_string "info" "Applied $label successfully (stash kept at $ref for safety)"
      print_string "warning" "Please resolve any conflicts and drop the stash manually if needed"
      return 0
    else
      print_string "warning" "Failed to auto-restore $label from $ref"
      return 1
    fi
  fi
  return 0
}

cleanup_on_exit() {
  if [ "$SUCCESS" = true ]; then
    return 0
  fi

  # 실패 시 자동 복구 시도. merge 중이면 stash 유지.
  # 커밋 rollback
  if [ "$LOCAL_SCHEMA_COMMITTED" = true ] && [ -n "$LAST_COMMIT_SHA" ]; then
    CURRENT_HEAD="$(git rev-parse HEAD)"
    if [ "$CURRENT_HEAD" = "$LAST_COMMIT_SHA" ]; then
      print_string "warning" "Rolling back failed commit..."
      git reset --soft HEAD~1
      print_string "info" "Commit rolled back. Changes are staged."
    fi
  fi

  restore_stash "$OUTSIDE_STASH_HASH" "non-schema changes" || true
  restore_stash "$SCHEMA_STASH_HASH" "$SCHEMA_DIR changes" || true
}

trap cleanup_on_exit EXIT

print_string "info" "=== STEP 1: Pull $REMOTE_NAME/main into $SCHEMA_DIR ==="

print_string "info" "Stashing local $SCHEMA_DIR changes (if any)..."
SCHEMA_STASH_HASH="$(stash_if_changed "schema-update-schema-stash-$(date +%s)" "$SCHEMA_DIR")"
if [ -n "$SCHEMA_STASH_HASH" ]; then
  print_string "info" "Stashed $SCHEMA_DIR changes"
fi

print_string "info" "Stashing non-schema changes (if any)..."
OUTSIDE_STASH_HASH="$(stash_if_changed "schema-update-outside-stash-$(date +%s)")"
if [ -n "$OUTSIDE_STASH_HASH" ]; then
  print_string "info" "Stashed non-schema changes"
fi

if [ -n "$(git status --porcelain=v1 -uall)" ]; then
  print_string "error" "Working tree is not clean after stashing"
  exit 1
fi

print_string "info" "Fetching $REMOTE_NAME/main..."
git fetch "$REMOTE_NAME" main

# 에디터 의존 제거
if ! GIT_EDITOR=true git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash -m "Merge $REMOTE_NAME/main into $SCHEMA_DIR"; then
  print_string "error" "subtree pull failed"

  # 충돌 상세 정보 표시
  if [ -f "$(git rev-parse --git-dir)/MERGE_HEAD" ]; then
    print_string "info" "Conflicted files:"
    git diff --name-only --diff-filter=U | while read -r file; do
      print_string "warning" "  - $file"
    done
    echo ""
    print_string "info" "To resolve:"
    print_string "info" "  1. Fix conflicts in the files above"
    print_string "info" "  2. git add <resolved-files>"
    print_string "info" "  3. git commit"
    print_string "info" "  4. Run this script again"
  fi

  exit 1
fi

print_string "success" "Pulled successfully"

# schema 변경 복원 후 schema 디렉토리만 커밋
if ! restore_stash "$SCHEMA_STASH_HASH" "$SCHEMA_DIR changes"; then
  print_string "error" "Failed to restore local $SCHEMA_DIR changes"
  exit 1
fi
SCHEMA_STASH_HASH=""

print_string "info" "=== STEP 2: Commit and push main repo ($SCHEMA_DIR only) ==="

if [ -n "$(git status --porcelain=v1 -uall -- "$SCHEMA_DIR")" ]; then
  echo ""
  git status --short -uall -- "$SCHEMA_DIR"
  echo ""
  git add -A -- "$SCHEMA_DIR"
  git commit -m "Update $SCHEMA_DIR from $REMOTE_NAME/main"
  LAST_COMMIT_SHA="$(git rev-parse HEAD)"

  if ! git push origin "$CURRENT_BRANCH"; then
    print_string "error" "Push failed"
    print_string "warning" "Rolling back commit..."
    git reset --soft HEAD~1
    print_string "info" "Commit rolled back. Changes are staged in $SCHEMA_DIR"
    print_string "info" "Fix the issue (e.g., pull remote changes) and run the script again"
    exit 1
  fi

  LOCAL_SCHEMA_COMMITTED=true
  print_string "success" "Pushed to origin/$CURRENT_BRANCH"
else
  print_string "info" "No local $SCHEMA_DIR changes after pull"
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

# 마지막에 non-schema 변경 복원
if ! restore_stash "$OUTSIDE_STASH_HASH" "non-schema changes"; then
  print_string "warning" "Non-schema changes were not auto-restored"
  print_string "info" "Restore manually with: git stash list"
  SUCCESS=true
  exit 0
fi
OUTSIDE_STASH_HASH=""

SUCCESS=true
if [ "$LOCAL_SCHEMA_COMMITTED" = true ]; then
  print_string "success" "Done"
else
  print_string "success" "Done (no local schema commit)"
fi
