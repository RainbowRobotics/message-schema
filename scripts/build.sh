#!/bin/bash

# SLAMNAV2 빌드 스크립트

# 스크립트 위치 확인
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# 프로젝트 루트는 scripts 폴더의 상위 폴더
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
BUILD_DIR="${PROJECT_ROOT}/build"

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}SLAMNAV2 Build Script${NC}"
echo -e "${GREEN}========================================${NC}"

# 빌드 타입 선택 (기본값: Release)
BUILD_TYPE=${1:-Release}

if [ "$BUILD_TYPE" != "Release" ] && [ "$BUILD_TYPE" != "Debug" ]; then
    echo -e "${YELLOW}Warning: Invalid build type '$BUILD_TYPE'. Using 'Release'${NC}"
    BUILD_TYPE="Release"
fi

echo -e "${YELLOW}Build Type: ${BUILD_TYPE}${NC}"

# build 디렉토리 생성
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

# CMake 설정
echo -e "${YELLOW}Running CMake...${NC}"
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE "$PROJECT_ROOT" || {
    echo -e "${RED}CMake configuration failed!${NC}"
    exit 1
}

# 빌드
echo -e "${YELLOW}Building...${NC}"
make -j$(nproc) || {
    echo -e "${RED}Build failed!${NC}"
    exit 1
}

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "Run the application with:"
echo -e "  ${YELLOW}${SCRIPT_DIR}/run.sh${NC}"
echo ""
echo -e "Or directly:"
echo -e "  ${YELLOW}${PROJECT_ROOT}/bin/SLAMNAV2${NC}"
