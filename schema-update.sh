#!/usr/bin/env bash
set -euo pipefail

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

SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

while [ $# -gt 0 ]; do
    case $1 in
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

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" || ! git diff --cached --quiet -- "$SCHEMA_DIR" || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "Changes in $SCHEMA_DIR"
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
    CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "Push failed"
        exit 1
    fi
    print_string "success" "Committed and pushed"
    SCHEMA_COMMITTED=true
else
    print_string "info" "No changes"
fi

echo ""
print_string "info" "=== STEP 1: Pull ==="

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
print_string "info" "Fetching..."
git fetch "$REMOTE_NAME" "main"

REMOTE_TREE="$(git rev-parse "refs/remotes/$REMOTE_NAME/main:$SCHEMA_DIR" 2>/dev/null || echo "")"
LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || echo "")"

if [ -z "$REMOTE_TREE" ]; then
    print_string "warning" "No remote schemas"
elif [ "$REMOTE_TREE" = "$LOCAL_TREE" ]; then
    print_string "success" "Up to date"
else
    print_string "warning" "New changes"
    echo ""
    NEED_STASH=false
    if ! git diff-index --quiet HEAD --; then
        print_string "info" "Stashing..."
        git stash push -m "auto-stash" -- . ":!$SCHEMA_DIR"
        NEED_STASH=true
    fi
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
        if [ "$NEED_STASH" = true ]; then
            print_string "warning" "Stashed changes exist"
            print_string "info" "Restore: git stash pop"
        fi
        exit 1
    fi
    if [ "$NEED_STASH" = true ]; then
        print_string "info" "Restoring..."
        git stash pop
    fi
fi

echo ""
print_string "info" "=== STEP 2: Commit ==="

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
else
    print_string "info" "No changes in working tree"
fi

# STEP 3는 SCHEMA_COMMITTED가 true일 때만 실행
if [ "$SCHEMA_COMMITTED" = false ]; then
    print_string "info" "No schemas commits, skipping STEP 3"
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

LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR")"
MAIN_COMMIT="$(git rev-parse --short HEAD)"

echo "Branch: $BR"
echo "Tree: $LOCAL_TREE"
echo "Commit: $MAIN_COMMIT"
echo ""

git fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

if git show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_TREE="$(git rev-parse "$REMOTE_NAME/$BR^{tree}")"
    echo "Remote tree: $REMOTE_TREE"
    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        print_string "info" "No changes"
        exit 0
    fi
    WORK_DIR="$(mktemp -d)"
    trap "git worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT
    git worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"
    (
        cd "$WORK_DIR"
        print_string "info" "Updating..."
        TEMP_EXTRACT="$(mktemp -d)"
        trap "rm -rf '$TEMP_EXTRACT'" EXIT
        git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x -C "$TEMP_EXTRACT"
        rsync -av "$TEMP_EXTRACT/" ./
        rm -rf "$TEMP_EXTRACT"
        git add -A
        if git diff --staged --quiet; then
            print_string "info" "No changes"
        else
            echo "Changed:"
            git diff --staged --name-status
            echo ""
            git commit -m "Update schemas from main @ $MAIN_COMMIT"
            git push "$REMOTE_NAME" "HEAD:refs/heads/$BR"
        fi
    ) || {
        print_string "error" "Push failed"
        exit 1
    }
else
    print_string "info" "Creating branch"

    # 이 브랜치만 타겟팅해서 정리
    if git show-ref --verify --quiet "refs/heads/$BR"; then
        # 이 브랜치를 사용하는 worktree만 찾아서 제거
        BRANCH_WORKTREES=$(git worktree list --porcelain | grep -A 3 "branch refs/heads/$BR" | grep "^worktree" | cut -d' ' -f2 || true)
        if [ -n "$BRANCH_WORKTREES" ]; then
            print_string "warning" "Found worktrees using branch $BR, cleaning up..."
            echo "$BRANCH_WORKTREES" | while read -r wt; do
                if [ -n "$wt" ]; then
                    print_string "info" "Removing: $wt"
                    git worktree remove --force "$wt" 2>/dev/null || true
                fi
            done
        fi
        # 브랜치 삭제 재시도
        if ! git branch -D "$BR" 2>/dev/null; then
            print_string "error" "Cannot delete branch $BR"
            print_string "info" "Please run: git worktree list"
            print_string "info" "And manually remove worktrees if needed"
            exit 1
        fi
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

print_string "success" "Complete"
