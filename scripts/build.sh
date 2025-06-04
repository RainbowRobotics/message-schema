#!/usr/bin/env bash
set -euo pipefail

##############################################################################
# 0. 경로 설정
##############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$ROOT_DIR/build"
PROJECT_FILE="$ROOT_DIR/SLAMNAV2.pro"
TARGET_NAME="SLAMNAV2"
LIB_DIR="$BUILD_DIR/lib"

echo "[INFO] 스크립트 경로: $SCRIPT_DIR"
echo "[INFO] 프로젝트 루트: $ROOT_DIR"

##############################################################################
# 1. 빌드 디렉토리 생성 및 빌드
##############################################################################
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "[INFO] 빌드 시작 (qmake + make)"
qmake "$PROJECT_FILE" -spec linux-g++ CONFIG+=release
make -j"$(nproc)"
echo "[OK] 빌드 완료"

##############################################################################
# 2. 실행 파일 탐색
##############################################################################
echo "[INFO] 실행 파일 검색 중..."
EXEC_PATH="$(find . -maxdepth 2 -type f -name "$TARGET_NAME" -perm -u=x | head -n 1)"
if [[ -z "$EXEC_PATH" ]]; then
    echo "❌ Error: '$TARGET_NAME' 실행 파일을 찾을 수 없습니다."
    exit 1
fi
echo "      → 실행파일 위치: $EXEC_PATH"

##############################################################################
# 3. ldd로 의존성 추출
##############################################################################
echo "[INFO] 의존성 분석 중 (ldd)"
mkdir -p "$LIB_DIR"

mapfile -t LIBS < <(
    ldd "$EXEC_PATH" | awk '
        ($2 == "=>") && ($3 !~ /not/) { print $3; next }
        ($1 ~ /^\//)                  { print $1 }
    ' | sort -u
)

##############################################################################
# 4. .so 복사 및 strip 전후 비교
##############################################################################
echo "[INFO] 의존 라이브러리 복사 및 strip 적용"

for lib in "${LIBS[@]}"; do
    dest="$LIB_DIR/$(basename "$lib")"
    if [[ -f "$dest" ]]; then
        echo "  ↳ $(basename "$lib") (이미 있음, 건너뜀)"
        continue
    fi

    cp -L --preserve=links "$lib" "$dest"

    before_size=$(stat -c %s "$dest")
    strip --strip-unneeded "$dest" || true
    after_size=$(stat -c %s "$dest")

    echo "  ↳ $(basename "$lib"): $(numfmt --to=iec $before_size) → $(numfmt --to=iec $after_size)"
done

echo "[✅ 완료] 실행파일 및 의존 라이브러리 준비 완료"
