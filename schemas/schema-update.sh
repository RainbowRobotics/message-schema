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

if [ "$MAIN_REPO" != "$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo '')" ]; then
  print_string "error" "Run from subtree only"
  exit 1
fi

if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
  print_string "error" "Not found: $SCHEMA_DIR"
  exit 1
fi

cd "$MAIN_REPO"

git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  print_string "error" "Remote not found: $REMOTE_NAME"
  exit 1
}

print_string "info" "=== Auto-commit ==="

SCHEMA_COMMITTED=false
SCHEMA_STASHED=false
LOCAL_SCHEMA_COMMITTED=false
LOCAL_SCHEMA_CHANGED_LIST="$(mktemp "${TMPDIR:-/tmp}/schema-update-local-changed.XXXXXX")"
trap 'rm -f "$LOCAL_SCHEMA_CHANGED_LIST" >/dev/null 2>&1 || true' EXIT

# subtree pull은 전체 워킹트리가 clean해야 안전하게 동작한다.
if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
    print_string "error" "Staged changes exist outside $SCHEMA_DIR"
    print_string "info" "Please commit/stash non-schema files first"
    exit 1
fi

if [ -n "$(git status --porcelain -- . ":!$SCHEMA_DIR")" ]; then
    print_string "error" "Working tree changes exist outside $SCHEMA_DIR"
    print_string "info" "Please commit/stash non-schema files first"
    exit 1
fi

# pull 전에 사용자가 실제로 건드린 schema 파일 목록을 저장한다.
git status --porcelain -- "$SCHEMA_DIR" \
    | sed -E 's/^.. //' \
    | sed -E 's/^[^ ]+ -> //' \
    > "$LOCAL_SCHEMA_CHANGED_LIST"

echo ""
print_string "info" "=== STEP 1: Pull ==="

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" || ! git diff --cached --quiet -- "$SCHEMA_DIR" || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "Stashing local $SCHEMA_DIR changes before pull"
    git stash push -u -m "schema-update-schema-stash" -- "$SCHEMA_DIR"
    SCHEMA_STASHED=true
fi

print_string "info" "Fetching..."
git fetch "$REMOTE_NAME" "main"

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

has_unmerged() {
  git diff --name-only --diff-filter=U | grep -q .
}

restore_one_stash() {
  local hash="$1"
  local label="$2"
  [ -z "$hash" ] && return 0

  local ref
  ref="$(stash_ref_from_hash "$hash")"
  if [ -z "$ref" ]; then
    return 0
  fi

  if has_unmerged; then
    print_string "warning" "$label stash kept ($ref) because repository has merge conflicts"
    return 1
  fi

  print_string "info" "Restoring $label changes from $ref..."
  if ! git stash pop "$ref"; then
    print_string "warning" "Failed to auto-restore $label changes from $ref"
    return 1
  fi
  return 0
}

cleanup_on_exit() {
  if [ "$SUCCESS" = true ]; then
    return 0
  fi

  # 실패 시 가능한 경우 자동 복구 시도 (outside 먼저, schema 나중)
  restore_one_stash "$OUTSIDE_STASH_HASH" "non-schema" || true
  restore_one_stash "$SCHEMA_STASH_HASH" "$SCHEMA_DIR" || true
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

if ! git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash; then
  print_string "error" "subtree pull conflict"
  print_string "warning" "Resolve conflicts and commit first, then run make schema-update again"
  exit 1
fi

print_string "success" "Pulled successfully"

# schema 작업 복원 후 schema 변경만 커밋
if ! restore_one_stash "$SCHEMA_STASH_HASH" "$SCHEMA_DIR"; then
  print_string "error" "Failed to restore local $SCHEMA_DIR changes"
  exit 1
fi
SCHEMA_STASH_HASH=""

print_string "info" "=== STEP 2: Commit and push only $SCHEMA_DIR ==="

if [ -z "$(git status --porcelain=v1 -uall -- "$SCHEMA_DIR")" ]; then
  print_string "info" "No changes in $SCHEMA_DIR"
else
    print_string "warning" "New changes"
    echo ""
    print_string "info" "Pulling..."
    if git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" "main" --squash; then
        print_string "success" "Pulled"
        if git push origin "$CURRENT_BRANCH"; then
            print_string "success" "Pushed"
        else
            print_string "warning" "Push failed"
        fi
        SCHEMA_COMMITTED=true
    else
        print_string "warning" "Conflict detected. Auto-resolving non-local files with theirs(main)..."

        AUTO_RESOLVED=0
        UNRESOLVED_LOCAL=0

        while IFS= read -r conflict_file; do
            [ -z "$conflict_file" ] && continue

            if grep -Fxq "$conflict_file" "$LOCAL_SCHEMA_CHANGED_LIST"; then
                print_string "warning" "Keep manual resolve: $conflict_file"
                UNRESOLVED_LOCAL=1
                continue
            fi

            print_string "info" "Auto-resolve with theirs: $conflict_file"
            git checkout --theirs -- "$conflict_file" >/dev/null 2>&1 || true
            git add -A -- "$conflict_file"
            AUTO_RESOLVED=$((AUTO_RESOLVED + 1))
        done < <(git diff --name-only --diff-filter=U -- "$SCHEMA_DIR")

        if [ "$UNRESOLVED_LOCAL" -eq 0 ]; then
            if git diff --name-only --diff-filter=U -- "$SCHEMA_DIR" | grep -q .; then
                print_string "error" "Conflict remains after auto-resolve"
            else
                print_string "success" "Auto-resolved $AUTO_RESOLVED file(s)"
                git commit -m "Merge subtree updates from $REMOTE_NAME/main"
                if git push origin "$CURRENT_BRANCH"; then
                    print_string "success" "Pushed"
                else
                    print_string "warning" "Push failed"
                fi
                SCHEMA_COMMITTED=true
                UNRESOLVED_LOCAL=0
            fi
        fi

        if [ "$UNRESOLVED_LOCAL" -eq 0 ] && ! git diff --name-only --diff-filter=U -- "$SCHEMA_DIR" | grep -q .; then
            :
        else
            print_string "error" "Conflict"
            echo ""
            print_string "warning" "=== Resolve in VSCode ==="
            git status --short | grep "^UU\|^AA\|^DD" || echo ""
            echo ""
            echo "1. Open VSCode Source Control"
            echo "2. Resolve conflicts"
            echo "3. git add schemas/"
            echo "4. git commit"
            echo "5. git push origin $CURRENT_BRANCH"
            echo "6. Run again: make schema-update"
            echo ""
            if [ "$SCHEMA_STASHED" = true ]; then
                print_string "warning" "Your schema changes are stashed"
                print_string "info" "Restore: git stash pop"
            fi
            exit 1
        fi
    fi
fi

if [ "$SCHEMA_STASHED" = true ]; then
    print_string "info" "Restoring local $SCHEMA_DIR changes..."
    if ! git stash pop; then
        print_string "error" "Failed to apply stashed schema changes"
        print_string "info" "Resolve conflicts, then rerun make schema-update"
        exit 1
    fi
fi

  git add -A -- "$SCHEMA_DIR"
  git commit -m "Update $SCHEMA_DIR from $REMOTE_NAME/main"

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" || ! git diff --cached --quiet -- "$SCHEMA_DIR" || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "Changes detected"
    echo ""
    git status --short -- "$SCHEMA_DIR"
    echo ""
    git add "$SCHEMA_DIR"
    if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
        print_string "warning" "Other files staged"
        git reset HEAD "$SCHEMA_DIR"
        exit 1
    fi
    git commit -m "Update schemas"
    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "Push failed"
        exit 1
    fi
    print_string "success" "Committed"
    SCHEMA_COMMITTED=true
    LOCAL_SCHEMA_COMMITTED=true
else
    print_string "info" "No changes in working tree"
fi

# STEP 3는 로컬 schema 변경 커밋이 있을 때만 실행
if [ "$LOCAL_SCHEMA_COMMITTED" = false ]; then
    print_string "info" "No local schema changes to publish, skipping STEP 3"
    print_string "success" "Done"
    exit 0
fi

echo ""
print_string "info" "=== STEP 3: Update branch ==="

BR="schema/from-$(git config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
    print_string "error" "Cannot determine branch"
    exit 1
  fi

  print_string "success" "Committed and pushed to origin/$CURRENT_BRANCH"
fi

LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR")"
MAIN_COMMIT="$(git rev-parse --short HEAD)"

echo "Branch: $BR"
echo "Tree: $LOCAL_TREE"
echo "Commit: $MAIN_COMMIT"
echo ""

git fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

if git show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_HEAD="$(git rev-parse "$REMOTE_NAME/$BR")"
    REMOTE_TREE="$(git rev-parse "$REMOTE_NAME/$BR^{tree}")"
    echo "Remote tree: $REMOTE_TREE"
    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        print_string "info" "No changes"
        exit 0
    fi
    print_string "info" "Updating branch with fast-forward commit..."
    NEW_COMMIT="$(printf "Update schemas from main @ %s\n" "$MAIN_COMMIT" | git commit-tree "$LOCAL_TREE" -p "$REMOTE_HEAD")"
    if ! git push "$REMOTE_NAME" "$NEW_COMMIT:refs/heads/$BR"; then
        print_string "warning" "Remote branch changed during push. Fetch latest and retry."
        print_string "error" "Push failed"
        exit 1
    fi
else
    print_string "info" "Creating branch"
    if git show-ref --verify --quiet "refs/heads/$BR"; then
        git branch -D "$BR"
    fi
    TMP="$BR-tmp"
    git branch -D "$TMP" 2>/dev/null || true
    git subtree split --prefix="$SCHEMA_DIR" -b "$TMP"
    if ! git push "$REMOTE_NAME" "$TMP:refs/heads/$BR"; then
        print_string "error" "Failed"
        git branch -D "$TMP" 2>/dev/null || true
        exit 1
    fi
    git branch -D "$TMP"
fi
OUTSIDE_STASH_HASH=""

SUCCESS=true
print_string "success" "Done"
