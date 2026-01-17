#!/usr/bin/env bash
set -euo pipefail

# 기본값 (필요하면 옵션으로 덮어씀)
SCHEMA_DIR="schemas"
REMOTE_NAME="message-schema"

# 옵션 처리
while [[ $# -gt 0 ]]; do
  case $1 in
    --dir) SCHEMA_DIR="$2"; shift 2 ;;
    --remote) REMOTE_NAME="$2"; shift 2 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 현재 서브트리 환경에서 실행되는지 확인
if [ -d "$SCRIPT_DIR/.git" ]; then
  echo "Error: 이 스크립트는 부모 레포지토리(서브트리 컨텍스트)에서 실행해야 합니다."
  echo "message-schema 레포지토리 내에서 직접 실행하지 마세요."
  exit 1
fi

# remote 없으면 추가
git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

STASH_REF=""

# 작업 트리가 clean하지 않으면 stash 생성
if [[ -n "$(git status --porcelain)" ]]; then
  TOKEN="$(date +%s%N)"
  MSG="schema-sync auto-stash ${TOKEN}"

  git stash push -u -m "$MSG" >/dev/null 2>&1 || true

  # 내가 만든 stash 엔트리의 stash@{n}을 메시지로 찾아 고정
  STASH_REF="$(
    git stash list --format='%gd %s' | awk -v msg="$MSG" '$0 ~ msg {print $1; exit}'
  )"

  if [[ -z "$STASH_REF" ]]; then
    echo "Error: stash entry를 찾을 수 없습니다. (stash 생성 실패)"
    echo "git status:"
    git status -sb
    exit 1
  fi

  # stash로 치운 뒤에도 dirty면 subtree 실패 -> 중단
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "작업 트리가 clean하지 않습니다. git subtree pull를 실행할 수 없습니다."
    git status -sb
    exit 1
  fi
fi

git fetch "$REMOTE_NAME" main

git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main"

# stash 복원 (stash@{n}만 apply/drop)
if [[ -n "$STASH_REF" ]]; then
  if git stash apply "$STASH_REF" >/dev/null; then
    git stash drop "$STASH_REF" >/dev/null
  else
    echo "stash 적용 중 충돌이 발생했습니다. 충돌 해결 후 아래 실행:"
    echo "  git stash drop $STASH_REF"
    exit 1
  fi
fi