#!/bin/bash

# 빌드 디렉터리가 없으면 생성
mkdir -p build

cd build

# qmake로 프로젝트 구성 (Release 모드)
qmake ../SLAMNAV2.pro -spec linux-g++ CONFIG+=release
if [ $? -ne 0 ]; then
    echo "Error: qmake로 프로젝트 구성 중 오류가 발생했습니다."
    exit 1
fi

# make로 빌드 수행
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Error: 빌드 과정에서 오류가 발생했습니다 (컴파일 실패)."
    exit 1
fi

# 빌드 성공 메시지 출력
echo "빌드가 성공적으로 완료되었습니다."