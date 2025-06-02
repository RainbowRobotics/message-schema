#!/usr/bin/env bash
set -euo pipefail

##############################################################################
# 0. 경로 설정 – 스크립트 위치를 기준으로 절대경로 계산
##############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # …/app_slamnav2/scripts
ROOT_DIR="$(dirname "$SCRIPT_DIR")"                          # …/app_slamnav2
BUILD_DIR="$ROOT_DIR/build"
PROJECT_FILE="$ROOT_DIR/SLAMNAV2.pro"
TARGET_NAME="SLAMNAV2"
LIB_DIR="$BUILD_DIR/lib"

echo "[INFO] 스크립트 위치: $SCRIPT_DIR"
echo "[INFO] 프로젝트 루트: $ROOT_DIR"

##############################################################################
# 1. 빌드 디렉터리 생성 및 이동
##############################################################################
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

##############################################################################
# 2. qmake → make
##############################################################################
echo "[INFO] qmake → make 시작"
qmake "$PROJECT_FILE" -spec linux-g++ CONFIG+=release
make -j"$(nproc)"
echo "[OK] 빌드 완료"

##############################################################################
# 3. 실행 파일 찾기
##############################################################################
echo "[INFO] 실행 파일 검색"
EXEC_PATH="$(find . -maxdepth 2 -type f -name "$TARGET_NAME" -perm -u=x | head -n 1)"
if [[ -z "$EXEC_PATH" ]]; then
    echo "Error: '$TARGET_NAME' 실행 파일을 찾지 못했습니다."
    exit 1
fi
echo "      → $EXEC_PATH"

##############################################################################
# 4. 의존 라이브러리 복사
##############################################################################
echo "[INFO] ldd 의존성 추출 및 복사"
mkdir -p "$LIB_DIR"

mapfile -t LIBS < <(
    ldd "$EXEC_PATH" | awk '
        ($2 == "=>") && ($3 !~ /not/) { print $3; next }   # libxx.so => /path/libxx.so ...
        ($1 ~ /^\//)                  { print $1 }         # /lib64/ld-linux-x86-64.so.2 ...
    ' | sort -u
)

for lib in "${LIBS[@]}"; do
    cp -L --preserve=links "$lib" "$LIB_DIR/"
    echo "  ↳ $(basename "$lib")"
done

echo "[DONE] 모든 라이브러리를 '$LIB_DIR' 에 복사했습니다."
