#!/bin/bash

echo "[ZRAM] Starting ZRAM configuration..."

# 1. 기존 zram0이 설정되어 있으면 초기화
if swapon --show | grep -q zram0; then
    echo "[ZRAM] zram0 is already active. Disabling swap..."
    sudo swapoff /dev/zram0
    echo "[ZRAM] Resetting zram0..."
    echo 1 | sudo tee /sys/block/zram0/reset
fi

# 2. zram 모듈 로드
echo "[ZRAM] Loading zram module..."
sudo modprobe zram

# 3. 총 메모리에서 75% 계산
echo "[ZRAM] Calculating 75% of total RAM..."
mem_kb=$(grep MemTotal /proc/meminfo | awk '{print $2}')
zram_size=$((mem_kb * 1024 * 75 / 100))
echo "[ZRAM] Target zram size: $zram_size bytes"

# 4. 압축 알고리즘 설정
echo "[ZRAM] Setting compression algorithm to lz4..."
echo lz4 | sudo tee /sys/block/zram0/comp_algorithm

# 5. 디스크 크기 설정
echo "[ZRAM] Setting zram0 disksize..."
echo $zram_size | sudo tee /sys/block/zram0/disksize

# 6. 스왑 포맷 및 활성화
echo "[ZRAM] Initializing swap on zram0..."
sudo mkswap /dev/zram0
sudo swapon --priority 100 /dev/zram0

# 7. 확인 출력
echo "[ZRAM] Setup complete. Status:"
swapon --show

