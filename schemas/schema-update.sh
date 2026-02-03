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

while [[ $# -gt 0 ]]; do
    case $1 in
        --dir) SCHEMA_DIR="$2"; shift 2 ;;
        --remote) REMOTE_NAME="$2"; shift 2 ;;
        *) print_string "error" "Unknown option: $1"; exit 1 ;;
    esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$MAIN_REPO" != "$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo '')" ]; then
    print_string "error" "This script must be run from a subtree"
    exit 1
fi

if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
    print_string "error" "Directory not found: $SCHEMA_DIR"
    exit 1
fi

cd "$MAIN_REPO"

git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
    print_string "error" "Remote not found: $REMOTE_NAME"
    exit 1
}

print_string "info" "=== Auto-commit main repo changes ==="

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" \
  || ! git diff --cached --quiet -- "$SCHEMA_DIR" \
  || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "Changes detected in $SCHEMA_DIR"
    echo ""
    git status --short -- "$SCHEMA_DIR"
    echo ""

    git add "$SCHEMA_DIR"

    if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
        print_string "warning" "Other files staged. Aborting."
        git reset HEAD "$SCHEMA_DIR"
        exit 1
    fi

    git commit -m "Update schemas"

    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "Push failed"
        exit 1
    fi

    print_string "success" "Main repo committed and pushed"
else
    print_string "info" "No changes in $SCHEMA_DIR"
fi

echo ""

print_string "info" "=== STEP 1: Pull from message-schema ==="

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

print_string "info" "Fetching $REMOTE_NAME/main..."
git fetch "$REMOTE_NAME" "main"

REMOTE_TREE=$(git rev-parse "refs/remotes/$REMOTE_NAME/main:$SCHEMA_DIR" 2>/dev/null || echo "")
LOCAL_TREE=$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || echo "")

if [ -z "$REMOTE_TREE" ]; then
    print_string "warning" "No $SCHEMA_DIR in $REMOTE_NAME"
elif [ "$REMOTE_TREE" = "$LOCAL_TREE" ]; then
    print_string "success" "Already up to date"
else
    print_string "warning" "New changes found in $REMOTE_NAME"
    echo ""

    NEED_STASH=false
    if ! git diff-index --quiet HEAD --; then
        print_string "info" "Stashing other changes..."
        git stash push -m "auto-stash-schema-update" -- . ":!$SCHEMA_DIR"
        NEED_STASH=true
    fi

    print_string "info" "Running subtree pull..."

    if git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" "main" --squash; then
        print_string "success" "Pulled from $REMOTE_NAME"

        if git push origin "$CURRENT_BRANCH"; then
            print_string "success" "Pushed to main repo"
        else
            print_string "warning" "Push failed"
        fi
    else
        print_string "error" "Subtree pull failed (conflict)"
        echo ""

        git merge --abort 2>/dev/null || true
        print_string "info" "Merge aborted"
        echo ""

        if [ "$NEED_STASH" = true ]; then
            print_string "info" "Restoring stashed changes..."
            git stash pop
        fi

        echo ""
        print_string "error" "=== Conflict with $REMOTE_NAME ==="
        echo ""
        print_string "info" "Resolve and retry:"
        echo "1. Check: git status"
        echo "2. Resolve conflicts"
        echo "3. git add $SCHEMA_DIR"
        echo "4. git commit"
        echo "5. git push origin $CURRENT_BRANCH"
        echo "6. Retry: make schema-update"
        exit 1
    fi

    if [ "$NEED_STASH" = true ]; then
        print_string "info" "Restoring stashed changes..."
        git stash pop
    fi
fi

echo ""

print_string "info" "=== STEP 2: Commit main repo changes ==="

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" \
  || ! git diff --cached --quiet -- "$SCHEMA_DIR" \
  || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then
    print_string "info" "Changes in $SCHEMA_DIR"
    echo ""
    git status --short -- "$SCHEMA_DIR"
    echo ""

    git add "$SCHEMA_DIR"

    if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
        print_string "warning" "Other files staged. Aborting."
        git reset HEAD "$SCHEMA_DIR"
        exit 1
    fi

    git commit -m "Update schemas"

    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "Push failed"
        exit 1
    fi

    print_string "success" "Main repo committed"
else
    print_string "info" "No changes in $SCHEMA_DIR"
    print_string "success" "Done (nothing to push)"
    exit 0
fi

echo ""

print_string "info" "=== STEP 3: Update message-schema branch ==="

BR="schema/from-$(git config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
    print_string "error" "Cannot determine branch name"
    exit 1
fi

LOCAL_TREE=$(git rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git rev-parse --short HEAD)

echo "schema branch => $BR"
print_string "info" "local  tree => $LOCAL_TREE"
print_string "info" "main commit => $MAIN_COMMIT"
echo ""

git fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

if git show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_TREE=$(git rev-parse "$REMOTE_NAME/$BR^{tree}")
    print_string "info" "remote tree => $REMOTE_TREE"

    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        print_string "info" "No changes"
        exit 0
    fi

    WORK_DIR=$(mktemp -d)
    trap "git worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT

    git worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"

    (
        cd "$WORK_DIR"

        print_string "info" "Updating schemas..."

        TEMP_EXTRACT=$(mktemp -d)
        trap "rm -rf '$TEMP_EXTRACT'" EXIT

        git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x -C "$TEMP_EXTRACT"

        rsync -av \
            "$TEMP_EXTRACT/" \
            ./

        rm -rf "$TEMP_EXTRACT"

        git add -A
        if git diff --staged --quiet; then
            print_string "info" "No changes"
        else
            print_string "info" "Changed files:"
            git diff --staged --name-status
            echo ""

            git commit -m "Update schemas from main repo @ $MAIN_COMMIT"
            git push "$REMOTE_NAME" "HEAD:refs/heads/$BR"
        fi
    ) || {
        print_string "error" "Push to $REMOTE_NAME failed"
        exit 1
    }
else
    print_string "info" "Creating new branch: $BR"

    if git show-ref --verify --quiet "refs/heads/$BR"; then
        print_string "warning" "Local branch exists. Deleting..."
        git branch -D "$BR"
    fi

    TMP="$BR-tmp"

    git branch -D "$TMP" 2>/dev/null || true
    git subtree split --prefix="$SCHEMA_DIR" -b "$TMP"

    if ! git push "$REMOTE_NAME" "$TMP:refs/heads/$BR"; then
        print_string "error" "Failed to create branch"
        git branch -D "$TMP" 2>/dev/null || true
        exit 1
    fi

    git branch -D "$TMP"
fi

print_string "success" "Push to $REMOTE_NAME/$BR complete"
