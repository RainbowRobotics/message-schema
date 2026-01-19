#!/usr/bin/env bash
set -euo pipefail

# 컬러 출력 함수
function print_string() {
  local RED=$'\033[0;31m'
  local GREEN=$'\033[0;32m'
  local YELLOW=$'\033[1;33m'
  local BLUE=$'\033[0;34m'
  local NC=$'\033[0m'

  case "$1" in
    error)   printf "%s%s%s\n" "$RED"   "$2" "$NC" ;;
    success) printf "%s%s%s\n" "$GREEN" "$2" "$NC" ;;
    warning) printf "%s%s%s\n" "$YELLOW""$2" "$NC" ;;
    info)    printf "%s%s%s\n" "$BLUE"  "$2" "$NC" ;;
    *)       printf "%s\n" "$2" ;;
  esac
}

# y/n 확인 유틸 (function 키워드 없이)
function confirm_or_exit() {
    local message="$1"
    echo ""
    print_string "warning" "${message}"
    echo ""
    read -r -p "계속 진행하시겠습니까? (y/N): " ANS
    case "${ANS,,}" in
        y|yes) return 0 ;;
        *) echo "취소되었습니다."; exit 0 ;;
    esac
}

# 기본값
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 옵션 처리
while [[ $# -gt 0 ]]; do
    case $1 in
        --dir) SCHEMA_DIR="$2"; shift 2 ;;
        --remote) REMOTE_NAME="$2"; shift 2 ;;
        *) print_string "error" "Unknown option: $1"; exit 1 ;;
    esac
done

# 메인 레포 루트
MAIN_REPO="$(git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 서브트리로 포함되었는지 확인 (독립 클론 방지)
if [ -d "$SCRIPT_DIR/.git" ]; then
    print_string "error" "이 스크립트는 서브트리 컨텍스트(메인 레포)에서 실행해야 합니다."
    exit 1
fi

cd "$MAIN_REPO"

# SCHEMA_DIR 존재 확인
if [ ! -d "$SCHEMA_DIR" ]; then
    print_string "error" "'$SCHEMA_DIR' 디렉토리를 찾을 수 없습니다."
    exit 1
fi

# remote 확인/추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

print_string "info" "DEBUG: Fetching $REMOTE_NAME/main"
git fetch "$REMOTE_NAME" main

# 로컬 schemas 트리(HEAD 기준) - subtree가 제대로 붙었는지 확인용
LOCAL_TREE="$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || true)"
if [ -z "${LOCAL_TREE:-}" ]; then
    print_string "error" "HEAD에 '$SCHEMA_DIR' 트리가 없습니다. subtree 초기 상태/경로를 확인하세요."
    exit 1
fi

# 원격 main 트리(현재 fetch 기준)
REMOTE_TREE_BEFORE="$(git rev-parse "$REMOTE_NAME/main^{tree}")"

print_string "info" "local  tree (HEAD:$SCHEMA_DIR) => $LOCAL_TREE"
print_string "info" "remote tree ($REMOTE_NAME/main) => $REMOTE_TREE_BEFORE"

# schemas의 로컬 변경(추가/수정/staged/untracked) 감지
HAS_CHANGES=0
if ! git diff --quiet -- "$SCHEMA_DIR"; then HAS_CHANGES=1; fi
if ! git diff --cached --quiet -- "$SCHEMA_DIR"; then HAS_CHANGES=1; fi
if [ -n "$(git ls-files --others --exclude-standard -- "$SCHEMA_DIR")" ]; then HAS_CHANGES=1; fi

# HEAD 기준 schemas 트리가 원격 main과 다른지(커밋되어 clean이어도 잡힘)
TREE_DIFF=0
if [ "$LOCAL_TREE" != "$REMOTE_TREE_BEFORE" ]; then
    TREE_DIFF=1
fi

# 이미 동일하고, 로컬 변경도 없으면 종료
if [ "$HAS_CHANGES" -eq 0 ] && [ "$TREE_DIFF" -eq 0 ]; then
    print_string "info" "schemas는 이미 $REMOTE_NAME/main과 동일합니다."
    exit 0
fi

confirm_or_exit \
"'$SCHEMA_DIR'을(를) '$REMOTE_NAME/main'과 동일하게 강제 동기화합니다.
 - '$SCHEMA_DIR' 아래의 로컬 수정/추가 파일은 모두 삭제됩니다. (untracked 포함)
 - staged 변경도 모두 폐기됩니다.
 - 원하지 않으신다면 취소 후 make schema-update를 통해 main 브랜치로 업데이트 후 다시 시도하세요.(PR이 팀원들에게 Apply 되어야 main 브랜치로 반영됩니다.)
 "

# schemas 워킹트리/인덱스 강제 초기화 (추적 파일 원복)
git restore -SW -- "$SCHEMA_DIR" 2>/dev/null || {
  print_string "error" "schemas 초기화(git restore) 실패"
  exit 1
}

# schemas 아래 untracked 파일/폴더 삭제
git clean -fd -- "$SCHEMA_DIR" || {
  print_string "error" "schemas 정리(git clean) 실패"
  exit 1
}

print_string "info" "DEBUG: About to subtree pull (force mirror)"
set +e
SUBTREE_OUT="$(git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main" 2>&1)"
RC=$?
set -e

print_string "info" "$SUBTREE_OUT"

if [ $RC -ne 0 ]; then
    echo "Error: subtree pull 실패 (exit code: $RC)"
    exit $RC
fi

# subtree pull 과정에서 원격 main이 더 최신으로 움직일 수 있으므로 재-fetch + 재계산
print_string "info" "Re-fetching $REMOTE_NAME/main for verification"
git fetch "$REMOTE_NAME" main

REMOTE_TREE_AFTER="$(git rev-parse "$REMOTE_NAME/main^{tree}")"
LOCAL_TREE_AFTER="$(git rev-parse "HEAD:$SCHEMA_DIR" 2>/dev/null || true)"

print_string "info" "local  tree after => ${LOCAL_TREE_AFTER:-<none>}"
print_string "info" "remote tree after => $REMOTE_TREE_AFTER"

if [ -z "$LOCAL_TREE_AFTER" ] || [ "$LOCAL_TREE_AFTER" != "$REMOTE_TREE_AFTER" ]; then
    print_string "error" "동기화 후에도 '$SCHEMA_DIR'이(가) '$REMOTE_NAME/main'과 동일하지 않습니다."
    print_string "info" "  local  => ${LOCAL_TREE_AFTER:-<none>}"
    print_string "info" "  remote => $REMOTE_TREE_AFTER"
    exit 1
fi

print_string "success" "schemas 동기화 완료 (message-schema/main과 동일)"
