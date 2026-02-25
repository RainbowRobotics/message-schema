#!/usr/bin/env bash
set -euo pipefail

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
  esac
}

SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"
AUTO_YES=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    -y|--yes) AUTO_YES=true; shift ;;
    *) print_string "error" "알 수 없는 옵션: $1"; exit 1 ;;
  esac
done

MAIN_REPO="$(git rev-parse --show-toplevel)"

cd "$MAIN_REPO"

git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || {
  print_string "error" "원격 저장소를 찾을 수 없습니다: $REMOTE_NAME"
  exit 1
}

print_string "info" "=== $REMOTE_NAME/main과 schemas 동기화 ==="
echo ""
print_string "warning" "${SCHEMA_DIR}의 모든 로컬 변경사항이 삭제됩니다"
print_string "warning" "$REMOTE_NAME/main 기준으로 subtree pull 됩니다"
echo ""
if [[ "$AUTO_YES" != true ]]; then
  read -r -p "계속하시겠습니까? (y/N): " ANS
  if [[ ! "$ANS" =~ ^[Yy]$ ]]; then
    print_string "info" "취소되었습니다"
    exit 0
  fi
fi

print_string "info" "$REMOTE_NAME/main을 가져오는 중..."
git fetch "$REMOTE_NAME" main

# subtree pull은 워킹 트리가 깨끗해야 한다.
if [[ -n "$(git status --porcelain -- . ":!$SCHEMA_DIR")" ]]; then
  print_string "error" "$SCHEMA_DIR 외 다른 파일 변경이 있어 동기화할 수 없습니다"
  print_string "info" "다른 변경을 먼저 commit/stash 해주세요"
  exit 1
fi

# schemas 내부 로컬 변경은 명시적으로 폐기한다.
if [[ -n "$(git status --porcelain -- "$SCHEMA_DIR")" ]]; then
  print_string "info" "$SCHEMA_DIR 로컬 변경을 삭제합니다..."
  git restore --staged --worktree -- "$SCHEMA_DIR" 2>/dev/null || true
  git checkout -- "$SCHEMA_DIR" 2>/dev/null || true
  git clean -fd -- "$SCHEMA_DIR"
fi

# subtree가 아직 없는 경우만 add
if [[ ! -d "$SCHEMA_DIR" ]]; then
  print_string "warning" "${SCHEMA_DIR}가 없어 subtree add를 수행합니다..."
  GIT_EDITOR=true git subtree add --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
    -m "Merge $REMOTE_NAME/main into $SCHEMA_DIR"
else
  print_string "info" "$REMOTE_NAME/main에서 $SCHEMA_DIR 업데이트 중..."
  GIT_EDITOR=true git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
    -m "Merge $REMOTE_NAME/main into $SCHEMA_DIR"
fi

print_string "success" "동기화 완료!"
print_string "info" "origin에 푸시 중..."

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if git push origin "$CURRENT_BRANCH"; then
  print_string "success" "푸시 완료!"
else
  print_string "warning" "푸시 실패. 수동으로 푸시하세요:"
  echo "  git push origin $CURRENT_BRANCH"
fi
