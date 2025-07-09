#!/usr/bin/env bash
# analyze_fault_logs.sh
# 모든 fault_log/*.txt 파일에 대해 addr2line 결과를 붙여서 출력.

set -euo pipefail

# SLAMNAV2 실행 파일 절대 경로 (Release 바이너리)
EXEC="/home/rainbow/code/app_slamnav2_bj_test-uper/app_slamnav2/SLAMNAV2"
# fault_log 디렉터리 절대 경로
LOGDIR="/home/rainbow/code/app_slamnav2_bj_test-uper/app_slamnav2/snlog/fault_log"


# 정규식: /path/to/module(+0xOFF)
regex='^([^[:space:]]+)\(\+0x([0-9a-fA-F]+)\)'

for logfile in "$LOGDIR"/*.txt; do
    [[ -e "$logfile" ]] || { echo "No *.txt in $LOGDIR"; exit 1; }

    echo "============================================================"
    echo " LOGFILE : $logfile"
    echo "============================================================"

    while IFS= read -r line; do
        if [[ $line =~ $regex ]]; then
            module="${BASH_REMATCH[1]}"
            offset="0x${BASH_REMATCH[2]}"

            # 모듈이 경로 없이 찍힌 경우 ─ 메인 실행 파일로 간주
            [[ $module == /* ]] || module="$EXEC"

            map_out=$(addr2line -f -C -e "$module" "$offset" 2>/dev/null)

            func=$(echo "$map_out" | head -n1)
            src=$(echo "$map_out"  | tail -n1)

            printf "%s\n    ↳ %s (%s)\n" "$line" "$func" "$src"
        else
            echo "$line"
        fi
    done < "$logfile"

    echo            # 로그 간 공백
done

