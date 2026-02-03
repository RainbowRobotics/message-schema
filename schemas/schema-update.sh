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
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            exit 0
            ;;
        *) print_string "error" "Unknown option: $1"; exit 1 ;;
    esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_string "info" "=== Environment Check ==="

if [ "$MAIN_REPO" != "$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo '')" ]; then
    print_string "error" "This script must be run from a subtree"
    exit 1
fi

if [ ! -d "$MAIN_REPO/$SCHEMA_DIR" ]; then
    print_string "error" "Directory '$SCHEMA_DIR' not found"
    exit 1
fi

cd "$MAIN_REPO"

if ! git remote get-url "$REMOTE_NAME" >/dev/null 2>&1; then
    print_string "error" "Remote '$REMOTE_NAME' not found"
    exit 1
fi

print_string "success" "Environment check passed"
echo ""

print_string "info" "=== Commit main repo changes first ==="

if ! git diff --quiet HEAD -- "$SCHEMA_DIR" \
  || ! git diff --cached --quiet -- "$SCHEMA_DIR" \
  || [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then

    print_string "warning" "Changes detected in $SCHEMA_DIR"
    echo ""
    git status --short -- "$SCHEMA_DIR"
    echo ""

    git add "$SCHEMA_DIR"

    if ! git diff --cached --quiet -- . ":!$SCHEMA_DIR"; then
        print_string "error" "Other files are staged besides $SCHEMA_DIR"
        git reset HEAD "$SCHEMA_DIR"
        exit 1
    fi

    git commit -m "Update schemas"

    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    if ! git push origin "$CURRENT_BRANCH"; then
        print_string "error" "Failed to push main repo"
        exit 1
    fi

    print_string "success" "Main repo committed and pushed"
else
    print_string "info" "No uncommitted changes in $SCHEMA_DIR"
fi

echo ""

print_string "info" "=== STEP 1: Pull from message-schema ==="

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

print_string "info" "Fetching message-schema/main..."
git fetch "$REMOTE_NAME" "main"

REMOTE_TREE=$(git rev-parse "refs/remotes/$REMOTE_NAME/main:$SCHEMA_DIR" 2>/dev/null || echo "")
LOCAL_TREE=$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || echo "")

if [ -z "$REMOTE_TREE" ]; then
    print_string "warning" "No schemas directory in message-schema"
elif [ "$REMOTE_TREE" = "$LOCAL_TREE" ]; then
    print_string "success" "Already up to date"
else
    print_string "warning" "New changes found in message-schema"
    echo ""

    # schemas 디렉토리는 clean해야 함
    if ! git diff --quiet HEAD -- "$SCHEMA_DIR" \
      || ! git diff --cached --quiet -- "$SCHEMA_DIR"; then
        print_string "error" "Uncommitted changes in $SCHEMA_DIR. This should not happen."
        exit 1
    fi

    # 다른 파일의 변경사항이 있으면 임시로 stash
    NEED_STASH=false
    if ! git diff-index --quiet HEAD --; then
        print_string "info" "Other files have changes, stashing temporarily..."
        git stash push -m "temp-for-schema-update" -- . ":!$SCHEMA_DIR"
        NEED_STASH=true
    fi

    print_string "info" "Running subtree pull..."

    if git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" "main" --squash; then
        print_string "success" "Pulled changes from message-schema"

        if git push origin "$CURRENT_BRANCH"; then
            print_string "success" "Pushed to main repo"
        else
            print_string "warning" "Failed to push to main repo"
        fi
    else
        print_string "error" "Subtree pull failed"

        # stash 복원
        if [ "$NEED_STASH" = true ]; then
            print_string "info" "Restoring stashed changes..."
            git stash pop
        fi

        exit 1
    fi

    # stash 복원
    if [ "$NEED_STASH" = true ]; then
        print_string "info" "Restoring stashed changes..."
        git stash pop
    fi
fi

echo ""

print_string "info" "=== STEP 2: Update message-schema branch ==="

USER_EMAIL=$(git config --get user.email)
USER_PREFIX=$(echo "$USER_EMAIL" | sed 's/@.*//' | tr -cd '[:alnum:]')

if [ -z "$USER_PREFIX" ]; then
    print_string "error" "Cannot determine user email"
    exit 1
fi

MY_BRANCH="schema/from-${USER_PREFIX}"
MAIN_COMMIT=$(git rev-parse --short HEAD)

echo "User          : $USER_PREFIX"
echo "Target branch : $MY_BRANCH"
echo "Main commit   : $MAIN_COMMIT"
echo ""

git fetch "$REMOTE_NAME"

BRANCH_EXISTS=false
if git show-ref --verify --quiet "refs/remotes/$REMOTE_NAME/$MY_BRANCH"; then
    BRANCH_EXISTS=true
    print_string "info" "Found existing branch: $MY_BRANCH"
else
    print_string "info" "No existing branch, will create new one"
fi

echo ""

print_string "info" "=== Prepare worktree ==="

WORK_DIR=$(mktemp -d)
trap "git worktree remove --force '$WORK_DIR' 2>/dev/null || true; rm -rf '$WORK_DIR'" EXIT

if [ "$BRANCH_EXISTS" = true ]; then
    git worktree add "$WORK_DIR" "$REMOTE_NAME/$MY_BRANCH"
    print_string "success" "Checked out existing branch"
else
    git worktree add --detach "$WORK_DIR" "refs/remotes/$REMOTE_NAME/main"
    (
        cd "$WORK_DIR"
        git checkout -b "$MY_BRANCH"
    )
    print_string "success" "Created new branch"
fi

echo ""

print_string "info" "=== Update schemas directory ==="

(
    cd "$WORK_DIR"

    TEMP_EXTRACT=$(mktemp -d)
    trap "rm -rf '$TEMP_EXTRACT'" EXIT

    LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")

    print_string "info" "Extracting schemas from main repo..."
    git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x -C "$TEMP_EXTRACT"

    print_string "info" "Updating schemas..."

    mkdir -p schemas

    rsync -av --delete \
        "$TEMP_EXTRACT/" \
        ./schemas/

    rm -rf "$TEMP_EXTRACT"

    git add schemas/

    if git diff --staged --quiet; then
        print_string "info" "No changes in schemas"
        exit 0
    fi

    echo ""
    print_string "info" "Changes in schemas:"
    git diff --staged --stat -- schemas/
    echo ""

    COMMIT_MSG="Update schemas from main repo

Source commit: $MAIN_COMMIT
Updated at: $(date -u +"%Y-%m-%d %H:%M:%S UTC")"

    git commit -m "$COMMIT_MSG"
    print_string "success" "Committed"
    echo ""

    print_string "info" "Pushing to $REMOTE_NAME/$MY_BRANCH..."

    if git push "$REMOTE_NAME" "$MY_BRANCH"; then
        print_string "success" "Push complete!"
        echo ""

        REMOTE_URL=$(git remote get-url "$REMOTE_NAME" 2>/dev/null || echo "")
        if [[ "$REMOTE_URL" =~ github\.com ]]; then
            REPO_URL=$(echo "$REMOTE_URL" | sed -E 's|git@github\.com:|https://github.com/|; s|\.git$||')

            if [ "$BRANCH_EXISTS" = true ]; then
                print_string "info" "Existing PR updated"
            else
                print_string "info" "Create PR:"
                echo "$REPO_URL/compare/main...$MY_BRANCH"
            fi
        fi
    else
        print_string "error" "Push failed"
        exit 1
    fi
) || {
    exit_code=$?
    if [ $exit_code -eq 0 ]; then
        exit 0
    fi
    print_string "error" "Update failed"
    exit 1
}

echo ""
print_string "success" "========================================="
print_string "success" "Sync complete!"
print_string "success" "========================================="
