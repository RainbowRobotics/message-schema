#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# 프로젝트 루트는 scripts 폴더의 상위 폴더
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_LIB_DIR="${PROJECT_ROOT}/build"
EXECUTABLE="${BIN_DIR}/SLAMNAV2"

if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: SLAMNAV2 실행파일을 찾을 수 없습니다: $EXECUTABLE"
    echo "먼저 빌드를 실행하세요:"
    echo "  ${SCRIPT_DIR}/build.sh"
    exit 1
fi

# 빌드된 공유 라이브러리들을 LD_LIBRARY_PATH에 추가
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/ui:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-common:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-communication:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-debug:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-navigation:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-slam:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-robot:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-map:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/lidar/RP:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/lidar/AIRY:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/lidar/SICK:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/lidar/LIVOX:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/lidar/LAKI:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${BUILD_LIB_DIR}/src/slamnav-sensors/drivers/cam/ORBBEC:${LD_LIBRARY_PATH}"

cd "$PROJECT_ROOT"

echo "Starting SLAMNAV2..."
echo "Working directory: $(pwd)"
echo "Executable: $EXECUTABLE"
echo "=========================================="

"$EXECUTABLE" "$@"
