#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BIN_DIR="${SCRIPT_DIR}/bin"
LIB_DIR="${BIN_DIR}/lib"
EXECUTABLE="${BIN_DIR}/SLAMNAV2"

if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: SLAMNAV2 실행파일을 찾을 수 없습니다: $EXECUTABLE"
    echo "먼저 빌드를 실행하세요:"
    echo "  cd build && cmake .. && make -j\$(nproc)"
    exit 1
fi

if [ -d "$LIB_DIR" ]; then
    export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH}"
fi

cd "$SCRIPT_DIR"

echo "Starting SLAMNAV2..."
echo "Working directory: $(pwd)"
echo "Executable: $EXECUTABLE"
echo "=========================================="

"$EXECUTABLE" "$@"
