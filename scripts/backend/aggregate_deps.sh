#!/usr/bin/env sh
set -eu

# === 설정 ===========================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "$ROOT"

SEARCH_DIRS="$ROOT/backend/packages $ROOT/backend/services"
OUT_IN="$ROOT/backend/requirements.in"
OUT_TXT="$ROOT/backend/requirements.txt"

# 내부(모노레포 로컬) 패키지 배포명(있으면 필터링)
# 예: "utils rb-zenoh flat-buffers app"
INTERNAL_NAMES="${INTERNAL_NAMES:-}"

# LOCK 전략: auto|uv|pip-compile|none
LOCK_WITH="${LOCK_WITH:-auto}"

# === 수집 ===========================================================
tmp="$(mktemp)"; trap 'rm -f "$tmp" "$tmp".*' EXIT

for base in $SEARCH_DIRS; do
  [ -d "$base" ] || continue
  # pyproject.toml 찾기
  find "$base" -type f -name pyproject.toml | while read -r f; do
    # 1) [project] 섹션 내부에서
    # 2) dependencies = [ ... ] 블록을 찾아
    # 3) 따옴표로 감싼 항목만 추출
    awk '
      BEGIN{inproj=0; indeps=0}
      /^[[:space:]]*\[project\][[:space:]]*$/ {inproj=1; next}
      /^[[:space:]]*\[[^]]+\][[:space:]]*$/ { if(inproj && !/^\s*\[project\]\s*$/) inproj=0 }
      {
        line=$0
        if(inproj && !indeps && line ~ /^[[:space:]]*dependencies[[:space:]]*=/){
          indeps=1
        }
        if(indeps){
          print line
          if(index(line, "]")>0){ indeps=0 }
        }
      }
    ' "$f" \
    | sed -e 's/#.*$//' \
          -e 's/^[[:space:]]*dependencies[[:space:]]*=[[:space:]]*\[//g' \
          -e 's/][[:space:]]*$//g' \
    | tr '\n' ' ' \
    | sed -E 's/[[:space:]]+/ /g' \
    | grep -Eo '"([^"\\]|\\.)*"|'\''([^'\''\\]|\\.)*'\''' \
    | sed -E 's/^["'\'']|["'\'']$//g' \
    >> "$tmp"
  done
done

# 내부 패키지 필터링(선택)
if [ -n "$INTERNAL_NAMES" ]; then
  pat="$(printf '%s\n' $INTERNAL_NAMES | tr '\n' '|' | sed 's/|$//')"
  # 배포명 정확 매칭만 제거
  grep -E -vi "^($(printf '%s' "$pat"))([=<>!~ ]|$)" "$tmp" > "$tmp.filtered" || true
  mv "$tmp.filtered" "$tmp"
fi

# 정렬·중복제거
sort -u "$tmp" > "$OUT_IN"
echo "✓ wrote $OUT_IN"

# === 잠금(선택) =====================================================
run_lock() {
  case "$1" in
    uv)
      command -v uv >/dev/null 2>&1 || return 1
      uv pip compile "$OUT_IN" -o "$OUT_TXT"
      ;;
    pip-compile)
      command -v pip-compile >/dev/null 2>&1 || return 1
      pip-compile "$OUT_IN" -o "$OUT_TXT"
      ;;
    none)
      cp "$OUT_IN" "$OUT_TXT"
      ;;
  esac
}

case "$LOCK_WITH" in
  auto)
    if run_lock uv; then echo "✓ locked with uv → $OUT_TXT"
    elif run_lock pip-compile; then echo "✓ locked with pip-compile → $OUT_TXT"
    else run_lock none; echo "⚠︎ no locker: copied $OUT_IN → $OUT_TXT"
    fi
    ;;
  uv|pip-compile|none)
    run_lock "$LOCK_WITH"
    echo "✓ locked with $LOCK_WITH → $OUT_TXT"
    ;;
  *)
    echo "LOCK_WITH must be one of: auto|uv|pip-compile|none" >&2; exit 2
    ;;
esac
