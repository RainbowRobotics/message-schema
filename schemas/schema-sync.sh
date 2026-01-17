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

git remote get-url "$REMOTE_NAME" >/dev/null 2>&1 || \
  git remote add "$REMOTE_NAME" "https://github.com/RainbowRobotics/message-schema.git"

STASH_REF=""
RESTORE_OK=0

restore_stash() {
  # stash를 만들었으면 무조건 원복 시도
  if [[ -n "$STASH_REF" ]]; then
    # apply 성공하면 drop
    if git stash apply "$STASH_REF" >/dev/null 2>&1; then
      git stash drop "$STASH_REF" >/dev/null 2>&1 || true
      RESTORE_OK=1
    else
      # 여기서는 exit 하지 말고, 메시지만 남긴다 (원인 파악용)
      echo "Warning: failed to apply $STASH_REF. Resolve manually."
      echo "  git stash list | head"
      echo "  git stash show -p $STASH_REF | less"
      echo "  (resolve) then: git stash drop $STASH_REF"
    fi
  fi
}

# 어떤 이유로든 스크립트가 끝나면 원복을 시도
trap restore_stash EXIT

# subtree pull은 clean 요구 → 더러우면 우리 stash를 하나 만든다
if [[ -n "$(git status --porcelain)" ]]; then
  TOKEN="$(date +%s%N)"
  MSG="schema-sync auto-stash ${TOKEN}"
  git stash push -u -m "$MSG" >/dev/null 2>&1 || true

  STASH_REF="$(git stash list --format='%gd %s' | awk -v msg="$MSG" '$0 ~ msg {print $1; exit}')"
  if [[ -z "$STASH_REF" ]]; then
    echo "Error: working tree dirty but stash was not created"
    git status -sb
    exit 1
  fi

  # stash 후에도 dirty면 subtree는 어차피 못 함
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "Error: still dirty after stash; cannot run subtree pull"
    git status -sb
    exit 1
  fi
fi

git fetch "$REMOTE_NAME" main

# 여기서부터는 'already at commit'인데 RC=1 나와도 죽지 않게 처리
set +e
SUBTREE_OUT="$(git subtree pull --prefix="$SCHEMA_DIR" "$REMOTE_NAME" main --squash \
  -m "Sync schemas from ${REMOTE_NAME}/main" 2>&1)"
RC=$?
set -e

printf "%s\n" "$SUBTREE_OUT"

# 이미 최신이면 성공 처리
if echo "$SUBTREE_OUT" | grep -q "Subtree is already at commit"; then
  RC=0
fi

# 진짜 실패면 종료 (EXIT trap이 stash 원복은 수행함)
exit "$RC"