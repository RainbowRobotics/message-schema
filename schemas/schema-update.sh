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
fi


# 개인 브랜치명 생성
BR="schema/from-$(git -C "$MAIN_REPO" config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"
if [ -z "$BR" ] || [ "$BR" = "schema/from-" ]; then
    print_string "error" "Cannot determine branch"
    exit 1
fi
echo "schema branch => $BR"

# 현재 상태
LOCAL_TREE=$(git -C "$MAIN_REPO" rev-parse "HEAD:$SCHEMA_DIR")
MAIN_COMMIT=$(git -C "$MAIN_REPO" rev-parse --short HEAD)

print_string "info" "local  tree => $LOCAL_TREE"
print_string "info" "main commit => $MAIN_COMMIT"

# remote 확인
git -C "$MAIN_REPO" remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
    print_string "error" "'$REMOTE_NAME' 원격 레포지토리를 찾을 수 없습니다."
    print_string "info" "message-schema 레포지토리에서 README.md에 초기 1회 세팅을 참고하세요."
    exit 1
}

# 원격 브랜치 fetch
git -C "$MAIN_REPO" fetch "$REMOTE_NAME" "+refs/heads/$BR:refs/remotes/$REMOTE_NAME/$BR" 2>/dev/null || true

REMOTE_REF="refs/remotes/$REMOTE_NAME/$BR"

# 기존 브랜치가 있는 경우
if git -C "$MAIN_REPO" show-ref --verify --quiet "$REMOTE_REF"; then
    REMOTE_TREE=$(git -C "$MAIN_REPO" rev-parse "$REMOTE_NAME/$BR^{tree}")
    print_string "info" "remote tree => $REMOTE_TREE"

    if [ "$LOCAL_TREE" = "$REMOTE_TREE" ]; then
        print_string "info" "No changes"
        exit 0
    fi

    WORK_DIR=$(mktemp -d)
    trap "git -C '$MAIN_REPO' worktree remove --force '$WORK_DIR' 2>/dev/null || true" EXIT

    git -C "$MAIN_REPO" worktree add --detach "$WORK_DIR" "$REMOTE_NAME/$BR"

    # 작업 디렉토리로 이동하여 처리
    (
        cd "$WORK_DIR"

        # [수정 포인트 1] 삭제 대상에서 제외할 목록 정의
        # .git은 당연히 제외, README와 .github도 명시적으로 제외함
        print_string "info" "스키마 파일 갱신 중 (메타 파일 보존)..."
        find . -mindepth 1 -maxdepth 1 \
            ! -name '.git' \
            ! -name '.github' \
            ! -name 'README.md' \
            ! -name '.gitignore' \
            -exec rm -rf {} +

        # [수정 포인트 2] 메인 레포의 SCHEMA_DIR 내용물만 가져오기
        git -C "$MAIN_REPO" archive "$LOCAL_TREE" | tar -x

        # 변경사항이 있는지 확인 후 커밋
        git add -A
        if git diff --staged --quiet; then
            print_string "info" "실질적인 변경사항이 없습니다."
        else
            git commit -m "Update schemas from main repo @ $MAIN_COMMIT"
            git push "$REMOTE_NAME" "HEAD:refs/heads/$BR"
            print_string "success" "Updated remote branch"
        fi
    ) || {
        print_string "error" "message-schema 푸시 실패"
        exit 1
    }
else
    print_string "info" "새 브랜치 생성: $BR"
    TMP="$BR-tmp"

    git -C "$MAIN_REPO" branch -D "$TMP" 2>/dev/null || true
    git -C "$MAIN_REPO" subtree split --prefix="$SCHEMA_DIR" -b "$TMP"

    if ! git -C "$MAIN_REPO" push "$REMOTE_NAME" "$TMP:refs/heads/$BR"; then
        print_string "error" "message-schema 초기 브랜치 생성 실패"
        git branch -D "$TMP" 2>/dev/null || true
        exit 1
    fi

    git -C "$MAIN_REPO" branch -D "$TMP"
fi

print_string "success" "Complete"
