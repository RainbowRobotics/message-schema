#!/usr/bin/env bash
set -euo pipefail

##############################################################################
# 0. 경로/상수 설정
##############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$ROOT_DIR/build"
TARGET_NAME="SLAMNAV2"
LIB_DIR="$BUILD_DIR/lib"

echo "[INFO] 스크립트 경로: $SCRIPT_DIR"
echo "[INFO] 프로젝트 루트 : $ROOT_DIR"

# .pro 자동 탐색
PROJECT_FILE=""
declare -a candidate_list=(
  "$ROOT_DIR/SLAMNAV2.pro"
  "$ROOT_DIR/slamnav2.pro"
  "$ROOT_DIR/src/SLAMNAV2.pro"
  "$ROOT_DIR/src/slamnav2.pro"
)

for f in "${candidate_list[@]}"; do
  if [[ -f "$f" ]]; then
    PROJECT_FILE="$f"
    break
  fi
done

if [[ -z "$PROJECT_FILE" ]]; then
  mapfile -t found < <(find "$ROOT_DIR" -maxdepth 2 -type f -name "*.pro" | sort)
  if [[ ${#found[@]} -gt 0 ]]; then
    PROJECT_FILE="${found[0]}"
  fi
fi

if [[ -z "$PROJECT_FILE" ]]; then
  echo "❌ 프로젝트 파일(.pro)을 찾지 못했습니다. ROOT 또는 src 아래에 *.pro가 있어야 합니다."
  exit 1
fi
echo "[INFO] 프로젝트 파일 : $PROJECT_FILE"

##############################################################################
# 1. 정리 → qmake 재생성 → make
##############################################################################
mkdir -p "$BUILD_DIR"

# (1) 이전 Makefile clean (build 디렉토리 기준)
if [[ -f "$BUILD_DIR/Makefile" ]]; then
  echo "[INFO] 이전 Makefile clean 실행"
  make -C "$BUILD_DIR" clean || true
fi

# (2) 이전 산물 혼선 제거
rm -f "$ROOT_DIR/$TARGET_NAME" "$BUILD_DIR/$TARGET_NAME" || true

# (3) qmake 캐시/임시물 정리
rm -rf "$BUILD_DIR"/* \
       "$ROOT_DIR/.qmake.stash" \
       "$ROOT_DIR/.qmake.cache" \
       "$ROOT_DIR/.moc" || true

# (4) qmake + make
cd "$BUILD_DIR"
echo "[INFO] 빌드 시작 (qmake + make)"
qmake -d "$PROJECT_FILE" -spec linux-g++ CONFIG+=release
make -j"$(nproc)"
echo "[✅ OK] 빌드 완료"

# (5) 루트에 생성된 실행파일이 있으면 build/로 이동 (DESTDIR=.. 대비)
if [[ -x "$ROOT_DIR/$TARGET_NAME" ]]; then
  echo "[INFO] 실행파일이 루트에 생성됨 → build/로 이동"
  mv -f "$ROOT_DIR/$TARGET_NAME" "$BUILD_DIR/$TARGET_NAME"
fi

##############################################################################
# 2. 실행 파일 찾기 (build 우선, 없으면 루트 보조)
##############################################################################
echo "[INFO] 실행 파일 검색 중…"
EXEC_PATH="$(find "$BUILD_DIR" "$ROOT_DIR" -maxdepth 2 -type f -name "$TARGET_NAME" -perm -u=x | head -n 1 || true)"
if [[ -z "$EXEC_PATH" ]]; then
  echo "❌ 실행 파일을 찾지 못했습니다."
  exit 1
fi
echo "      → $EXEC_PATH"

##############################################################################
# 3. ldd 재귀 수집 함수
##############################################################################
declare -A SEEN_LIBS
declare -a QUEUE
QUEUE+=("$EXEC_PATH")

gather_deps() {
  local bin="$1"
  ldd "$bin" 2>/dev/null | awk '
    /=>/ && $3 != "" && $3 != "not" { print $3; next }
    /^\//                           { print $1 }
  ' | grep -E '^/' | sort -u
}

while ((${#QUEUE[@]})); do
  cur="${QUEUE[0]}"
  QUEUE=("${QUEUE[@]:1}")
  mapfile -t deps < <(gather_deps "$cur")
  for dep in "${deps[@]}"; do
    [[ -z "${SEEN_LIBS[$dep]+x}" ]] || continue
    SEEN_LIBS["$dep"]=1
    QUEUE+=("$dep")
  done
done

##############################################################################
# 4. .so 복사 및 strip
##############################################################################
echo "[INFO] 의존 라이브러리 복사 및 strip 적용"
mkdir -p "$LIB_DIR"

for lib in "${!SEEN_LIBS[@]}"; do
  dest="$LIB_DIR/$(basename "$lib")"
  if [[ -f "$dest" ]]; then
    echo "  ↳ $(basename "$lib") (이미 있음, 건너뜀)"
    continue
  fi
  cp -L --preserve=links "$lib" "$dest" || { echo "  ⚠️ 복사 실패: $lib"; continue; }

  before=$(stat -c %s "$dest" 2>/dev/null || echo 0)
  strip --strip-unneeded "$dest" 2>/dev/null || true
  after=$(stat -c %s "$dest" 2>/dev/null || echo 0)

  if command -v numfmt >/dev/null 2>&1; then
    printf "  ↳ %-35s %7s → %s\n" "$(basename "$lib")" \
           "$(numfmt --to=iec $before)" "$(numfmt --to=iec $after)"
  else
    printf "  ↳ %-35s %7d → %d (bytes)\n" "$(basename "$lib")" "$before" "$after"
  fi
done

echo "[✅ OK] 모든 실행 파일·의존 라이브러리가 '$LIB_DIR'에 준비됐습니다."

##############################################################################
# 5. config 디렉토리 복사
##############################################################################
echo "[INFO] config 디렉토리 복사 중…"
if [[ -d "$ROOT_DIR/config" ]]; then
  rm -rf "$BUILD_DIR/config"
  cp -r "$ROOT_DIR/config" "$BUILD_DIR"
  echo "[✅ OK] config 디렉토리 복사 완료"
else
  echo "⚠️ '$ROOT_DIR/config' 디렉토리가 없습니다. 건너뜁니다."
fi

