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

while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
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
print_string "warning" "$SCHEMA_DIR의 모든 로컬 변경사항이 삭제됩니다"
print_string "warning" "$REMOTE_NAME/main의 내용으로 완전히 교체됩니다"
echo ""
read -r -p "계속하시겠습니까? (y/N): " ANS

if [[ ! "$ANS" =~ ^[Yy]$ ]]; then
  print_string "info" "취소되었습니다"
  exit 0
fi

print_string "info" "$REMOTE_NAME/main을 가져오는 중..."
git fetch "$REMOTE_NAME" main

# schemas 디렉토리 완전 삭제
print_string "info" "$SCHEMA_DIR 디렉토리 정리 중..."
rm -rf "$SCHEMA_DIR"
git add "$SCHEMA_DIR"
git commit -m "Clean schemas directory for sync"

# subtree pull로 최신 내용 가져오기 (add 대신 pull 사용)
print_string "info" "$REMOTE_NAME/main에서 $SCHEMA_DIR 동기화 중..."
if git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash; then
  print_string "success" "동기화 완료!"
else
  print_string "warning" "subtree pull 실패. subtree add 시도 중..."
  if git subtree add --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash; then
    print_string "success" "동기화 완료!"
  else
    print_string "error" "동기화 실패. 수동으로 처리가 필요합니다."
    exit 1
  fi
fi

print_string "info" "origin에 푸시 중..."

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if git push origin "$CURRENT_BRANCH"; then
  print_string "success" "푸시 완료!"
else
  print_string "warning" "푸시 실패. 수동으로 푸시하세요:"
  echo "  git push origin $CURRENT_BRANCH"
fi
